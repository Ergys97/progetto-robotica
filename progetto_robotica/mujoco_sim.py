#!/usr/bin/env python3
import os
import time
import yaml
import threading
import numpy as np
import torch
import mujoco
import mujoco.viewer

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Quaternion
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Bool, String
from nav_msgs.msg import Odometry

from progetto_robotica import sim_utils

class MuJoCoSimNode(Node):
    def __init__(self):
        super().__init__('mujoco_sim_node')
        
        # Declare parameters
        self.declare_parameter('xml_path', '/home/ergys/unitree_rl_gym/resources/robots/g1_description/scene.xml')
        self.declare_parameter('policy_path', '/home/ergys/unitree_rl_gym/deploy/pre_train/g1/motion.pt')
        self.declare_parameter('config_path', '/home/ergys/unitree_rl_gym/deploy/deploy_mujoco/configs/g1.yaml')
        self.declare_parameter('headless', False)
        
        xml_path = self.get_parameter('xml_path').value
        policy_path = self.get_parameter('policy_path').value
        config_path = self.get_parameter('config_path').value
        self.headless = self.get_parameter('headless').value
        
        self.get_logger().info(f"Loading config from: {config_path}")
        self.get_logger().info(f"Loading XML from: {xml_path}")
        self.get_logger().info(f"Loading policy from: {policy_path}")
        self.get_logger().info(f"Headless mode: {self.headless}")
        
        # Load configuration
        with open(config_path, 'r') as f:
            self.config = yaml.load(f, Loader=yaml.FullLoader)
            
        self.simulation_dt = self.config["simulation_dt"]
        self.control_decimation = self.config["control_decimation"]
        self.kps = np.array(self.config["kps"], dtype=np.float32)
        self.kds = np.array(self.config["kds"], dtype=np.float32)
        self.default_angles = np.array(self.config["default_angles"], dtype=np.float32)
        
        self.ang_vel_scale = self.config["ang_vel_scale"]
        self.dof_pos_scale = self.config["dof_pos_scale"]
        self.dof_vel_scale = self.config["dof_vel_scale"]
        self.action_scale = self.config["action_scale"]
        self.cmd_scale = np.array(self.config["cmd_scale"], dtype=np.float32)
        
        self.num_actions = self.config["num_actions"]
        self.num_obs = self.config["num_obs"]
        
        # Command velocities: [vx, vy, yaw_rate]
        self.cmd = np.array(self.config["cmd_init"], dtype=np.float32)
        
        # CSV telemetry log file variables
        self.csv_file = None
        self.csv_lock = threading.Lock()
        self.is_logging_csv = False
        
        # ROS 2 Subscribers
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.rec_status_sub = self.create_subscription(String, '/recording_status', self.rec_status_callback, 10)
        
        # ROS 2 Publishers
        self.imu_pub = self.create_publisher(Imu, '/imu', 10)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.left_contact_pub = self.create_publisher(Bool, '/contacts/left', 10)
        self.right_contact_pub = self.create_publisher(Bool, '/contacts/right', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        
        # Initialize MuJoCo model and data
        self.m = mujoco.MjModel.from_xml_path(xml_path)
        self.d = mujoco.MjData(self.m)
        self.m.opt.timestep = self.simulation_dt
        
        # Initialize RL policy
        self.policy = torch.jit.load(policy_path)
        
        # Dynamic extraction of foot geom IDs
        self.left_foot_geom_ids = []
        self.right_foot_geom_ids = []
        for i in range(self.m.ngeom):
            body_name = self.m.body(self.m.geom_bodyid[i]).name
            if body_name == 'left_ankle_roll_link':
                self.left_foot_geom_ids.append(i)
            elif body_name == 'right_ankle_roll_link':
                self.right_foot_geom_ids.append(i)
                
        self.get_logger().info(f"Left foot geoms: {self.left_foot_geom_ids}")
        self.get_logger().info(f"Right foot geoms: {self.right_foot_geom_ids}")
        
        self.joint_names = [
            'left_hip_pitch_joint', 'left_hip_roll_joint', 'left_hip_yaw_joint',
            'left_knee_joint', 'left_ankle_pitch_joint', 'left_ankle_roll_joint',
            'right_hip_pitch_joint', 'right_hip_roll_joint', 'right_hip_yaw_joint',
            'right_knee_joint', 'right_ankle_pitch_joint', 'right_ankle_roll_joint'
        ]
        
        # Start simulation loop in a separate thread
        self.sim_thread = threading.Thread(target=self.run_sim_loop)
        self.sim_thread.daemon = True
        self.sim_thread.start()
        
    def cmd_vel_callback(self, msg: Twist):
        # Update command: [vx, vy, yaw_rate]
        self.cmd[0] = msg.linear.x
        self.cmd[1] = msg.linear.y
        self.cmd[2] = msg.angular.z
        
    def rec_status_callback(self, msg: String):
        bag_name = msg.data
        with self.csv_lock:
            if bag_name:
                bag_dir = "/mnt/c/Users/ergys/Desktop/Git_Repositories/progetto-robotica/bags"
                os.makedirs(bag_dir, exist_ok=True)
                csv_path = os.path.join(bag_dir, f"{bag_name}.csv")
                self.get_logger().info(f"Opening CSV telemetry log: {csv_path}")
                try:
                    self.csv_file = open(csv_path, 'w')
                    # Write header
                    qpos_cols = ",".join([f"qpos_{i}" for i in range(self.m.nq)])
                    qvel_cols = ",".join([f"qvel_{i}" for i in range(self.m.nv)])
                    self.csv_file.write(f"timestamp,pos_x,pos_y,pos_z,cmd_vx,cmd_vy,cmd_wz,{qpos_cols},{qvel_cols}\n")
                    self.is_logging_csv = True
                except Exception as e:
                    self.get_logger().error(f"Failed to open CSV log file: {e}")
            else:
                if self.csv_file:
                    self.get_logger().info("Closing CSV telemetry log.")
                    self.is_logging_csv = False
                    self.csv_file.close()
                    self.csv_file = None
        

        
    def run_sim_loop(self):
        # Simulation loop variables
        action = np.zeros(self.num_actions, dtype=np.float32)
        target_dof_pos = self.default_angles.copy()
        obs = np.zeros(self.num_obs, dtype=np.float32)
        counter = 0
        
        if not self.headless:
            try:
                self.get_logger().info("Launching MuJoCo passive viewer...")
                with mujoco.viewer.launch_passive(self.m, self.d) as viewer:
                    while viewer.is_running() and rclpy.ok():
                        step_start = time.time()
                        
                        # Apply PD control
                        tau = sim_utils.pd_control(target_dof_pos, self.d.qpos[7:], self.kps, np.zeros_like(self.kds), self.d.qvel[6:], self.kds)
                        self.d.ctrl[:] = tau
                        
                        # Step physics
                        mujoco.mj_step(self.m, self.d)
                        
                        counter += 1
                        if counter % self.control_decimation == 0:
                            # Evaluate policy
                            self.evaluate_policy(obs, action, target_dof_pos, counter)
                            # Publish telemetries
                            self.publish_telemetries(counter)
                            
                        viewer.sync()
                        
                        # Real-time throttling
                        time_until_next_step = self.simulation_dt - (time.time() - step_start)
                        if time_until_next_step > 0:
                            time.sleep(time_until_next_step)
            except Exception as e:
                self.get_logger().error(f"MuJoCo viewer failed or closed: {e}. Falling back to headless simulation.")
                self.headless = True
                
        if self.headless:
            self.get_logger().info("Running headless simulation loop...")
            while rclpy.ok():
                step_start = time.time()
                
                # Apply PD control
                tau = sim_utils.pd_control(target_dof_pos, self.d.qpos[7:], self.kps, np.zeros_like(self.kds), self.d.qvel[6:], self.kds)
                self.d.ctrl[:] = tau
                
                # Step physics
                mujoco.mj_step(self.m, self.d)
                
                counter += 1
                if counter % self.control_decimation == 0:
                    # Evaluate policy
                    self.evaluate_policy(obs, action, target_dof_pos, counter)
                    # Publish telemetries
                    self.publish_telemetries(counter)
                    
                # Real-time throttling
                time_until_next_step = self.simulation_dt - (time.time() - step_start)
                if time_until_next_step > 0:
                    time.sleep(time_until_next_step)

    def evaluate_policy(self, obs, action, target_dof_pos, counter):
        qj = self.d.qpos[7:]
        dqj = self.d.qvel[6:]
        quat = self.d.qpos[3:7]
        omega = self.d.qvel[3:6]

        qj = (qj - self.default_angles) * self.dof_pos_scale
        dqj = dqj * self.dof_vel_scale
        gravity_orientation = sim_utils.get_gravity_orientation(quat)
        omega = omega * self.ang_vel_scale

        period = 0.8
        count = counter * self.simulation_dt
        phase = count % period / period
        sin_phase = np.sin(2 * np.pi * phase)
        cos_phase = np.cos(2 * np.pi * phase)

        obs[:3] = omega
        obs[3:6] = gravity_orientation
        obs[6:9] = self.cmd * self.cmd_scale
        obs[9 : 9 + self.num_actions] = qj
        obs[9 + self.num_actions : 9 + 2 * self.num_actions] = dqj
        obs[9 + 2 * self.num_actions : 9 + 3 * self.num_actions] = action
        obs[9 + 3 * self.num_actions : 9 + 3 * self.num_actions + 2] = np.array([sin_phase, cos_phase])
        obs_tensor = torch.from_numpy(obs).unsqueeze(0)
        
        # policy inference
        with torch.no_grad():
            new_action = self.policy(obs_tensor).numpy().squeeze()
            action[:] = new_action
            
        # transform action to target_dof_pos
        target_dof_pos[:] = action * self.action_scale + self.default_angles

    def publish_telemetries(self, counter):
        # Write to CSV log if active
        if self.is_logging_csv:
            with self.csv_lock:
                if self.csv_file:
                    t = time.time()
                    px = self.d.qpos[0]
                    py = self.d.qpos[1]
                    pz = self.d.qpos[2]
                    cvx = self.cmd[0]
                    cvy = self.cmd[1]
                    cwz = self.cmd[2]
                    qpos_str = ",".join([str(x) for x in self.d.qpos])
                    qvel_str = ",".join([str(x) for x in self.d.qvel])
                    self.csv_file.write(f"{t},{px},{py},{pz},{cvx},{cvy},{cwz},{qpos_str},{qvel_str}\n")
                    
        # 1. Contacts
        left_contact = False
        right_contact = False
        for i in range(self.d.ncon):
            c = self.d.contact[i]
            is_floor1 = (c.geom1 == 0 or self.m.geom(c.geom1).name == 'floor')
            is_floor2 = (c.geom2 == 0 or self.m.geom(c.geom2).name == 'floor')
            if is_floor1 or is_floor2:
                other_geom = c.geom2 if is_floor1 else c.geom1
                if other_geom in self.left_foot_geom_ids:
                    left_contact = True
                elif other_geom in self.right_foot_geom_ids:
                    right_contact = True
                    
        left_msg = Bool()
        left_msg.data = left_contact
        self.left_contact_pub.publish(left_msg)
        
        right_msg = Bool()
        right_msg.data = right_contact
        self.right_contact_pub.publish(right_msg)
        
        # 2. IMU
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "pelvis"
        
        # Convert MuJoCo [qw, qx, qy, qz] to ROS [qx, qy, qz, qw]
        imu_msg.orientation.w = float(self.d.qpos[3])
        imu_msg.orientation.x = float(self.d.qpos[4])
        imu_msg.orientation.y = float(self.d.qpos[5])
        imu_msg.orientation.z = float(self.d.qpos[6])
        
        # Angular velocity from pelvis free joint
        imu_msg.angular_velocity.x = float(self.d.qvel[3])
        imu_msg.angular_velocity.y = float(self.d.qvel[4])
        imu_msg.angular_velocity.z = float(self.d.qvel[5])
        
        # Linear acceleration in pelvis body frame (R.T @ (a + g))
        R = self.d.xmat[1].reshape(3, 3)
        acc_world = self.d.qacc[0:3] + np.array([0.0, 0.0, 9.81])
        acc_body = R.T @ acc_world
        imu_msg.linear_acceleration.x = float(acc_body[0])
        imu_msg.linear_acceleration.y = float(acc_body[1])
        imu_msg.linear_acceleration.z = float(acc_body[2])
        
        self.imu_pub.publish(imu_msg)
        
        # 3. Joint States
        js_msg = JointState()
        js_msg.header.stamp = self.get_clock().now().to_msg()
        js_msg.name = self.joint_names
        js_msg.position = self.d.qpos[7:].tolist()
        js_msg.velocity = self.d.qvel[6:].tolist()
        # Effort is optional, but we can compute it if needed
        js_msg.effort = self.d.ctrl[:].tolist()
        self.joint_pub.publish(js_msg)
        
        # 4. Odometry
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "pelvis"
        
        odom_msg.pose.pose.position.x = float(self.d.qpos[0])
        odom_msg.pose.pose.position.y = float(self.d.qpos[1])
        odom_msg.pose.pose.position.z = float(self.d.qpos[2])
        
        odom_msg.pose.pose.orientation.w = float(self.d.qpos[3])
        odom_msg.pose.pose.orientation.x = float(self.d.qpos[4])
        odom_msg.pose.pose.orientation.y = float(self.d.qpos[5])
        odom_msg.pose.pose.orientation.z = float(self.d.qpos[6])
        
        # Twist is child frame relative velocities
        vel_world = self.d.qvel[0:3]
        vel_body = R.T @ vel_world
        odom_msg.twist.twist.linear.x = float(vel_body[0])
        odom_msg.twist.twist.linear.y = float(vel_body[1])
        odom_msg.twist.twist.linear.z = float(vel_body[2])
        
        odom_msg.twist.twist.angular.x = float(self.d.qvel[3])
        odom_msg.twist.twist.angular.y = float(self.d.qvel[4])
        odom_msg.twist.twist.angular.z = float(self.d.qvel[5])
        
        self.odom_pub.publish(odom_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MuJoCoSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
