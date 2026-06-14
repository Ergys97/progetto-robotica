#!/usr/bin/env python3
import os
import time
import threading

import numpy as np
import torch
import yaml
import mujoco
import mujoco.viewer

import rclpy
import rclpy.time
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Bool, Empty, String, Float64
from nav_msgs.msg import Odometry

from progetto_robotica import scene_builder, sim_utils

DEFAULT_RL_GYM = os.path.expanduser(os.environ.get('UNITREE_RL_GYM_PATH', '~/unitree_rl_gym'))


class MuJoCoSimNode(Node):
    def __init__(self):
        super().__init__('mujoco_sim_node')

        self.declare_parameter('xml_path', os.path.join(DEFAULT_RL_GYM, 'resources/robots/g1_description/scene.xml'))
        self.declare_parameter('policy_path', os.path.join(DEFAULT_RL_GYM, 'deploy/pre_train/g1/motion.pt'))
        self.declare_parameter('config_path', os.path.join(DEFAULT_RL_GYM, 'deploy/deploy_mujoco/configs/g1.yaml'))
        self.declare_parameter('headless', False)
        self.declare_parameter('bag_dir', os.path.expanduser('~/progetto_robotica_bags'))
        self.declare_parameter('cmd_timeout', 0.5)
        self.declare_parameter('fall_threshold_deg', 35.0)
        self.declare_parameter('scenario', 'flat')

        xml_path = self.get_parameter('xml_path').value
        policy_path = self.get_parameter('policy_path').value
        config_path = self.get_parameter('config_path').value
        self.headless = self.get_parameter('headless').value
        self.bag_dir = os.path.expanduser(self.get_parameter('bag_dir').value)
        self.cmd_timeout = self.get_parameter('cmd_timeout').value
        self.fall_threshold_deg = self.get_parameter('fall_threshold_deg').value
        scenario = self.get_parameter('scenario').value

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

        # Stato di simulazione (attributi: servono per il reset deterministico)
        self.cmd = np.array(self.config["cmd_init"], dtype=np.float32)
        self.action = np.zeros(self.num_actions, dtype=np.float32)
        self.target_dof_pos = self.default_angles.copy()
        self.obs = np.zeros(self.num_obs, dtype=np.float32)
        self.counter = 0

        # Richieste asincrone gestite DAL thread di simulazione
        self.reset_requested = False
        self.pending_rec = None  # None = nulla; "" = stop; "<bag_name>" = start
        self.csv_file = None
        self.csv_lock = threading.Lock()
        self.is_logging_csv = False
        self.last_cmd_rx = time.monotonic()

        # ROS I/O
        self.cmd_vel_sub = self.create_subscription(TwistStamped, '/cmd_vel', self.cmd_vel_callback, 10)
        self.latency_pub = self.create_publisher(Float64, '/metrics/cmd_latency_ms', 10)
        self.rec_status_sub = self.create_subscription(String, '/recording_status', self.rec_status_callback, 10)
        self.reset_sub = self.create_subscription(Empty, '/sim_reset', self.reset_callback, 10)
        self.imu_pub = self.create_publisher(Imu, '/imu', 10)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.left_contact_pub = self.create_publisher(Bool, '/contacts/left', 10)
        self.right_contact_pub = self.create_publisher(Bool, '/contacts/right', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.fall_pub = self.create_publisher(Bool, '/fall_detected', 10)

        # MuJoCo
        generated_scene_dir = os.path.join(self.bag_dir, "_generated_scenes")
        xml_path = scene_builder.resolve_scene_xml(xml_path, scenario, generated_scene_dir)
        self.get_logger().info(f"Loading MuJoCo scenario '{scenario}' from: {xml_path}")
        self.m = mujoco.MjModel.from_xml_path(xml_path)
        self.d = mujoco.MjData(self.m)
        self.m.opt.timestep = self.simulation_dt
        self.policy = torch.jit.load(policy_path)

        self.pelvis_body_id = self.m.body('pelvis').id

        self.left_foot_geom_ids = []
        self.right_foot_geom_ids = []
        for i in range(self.m.ngeom):
            body_name = self.m.body(self.m.geom_bodyid[i]).name
            if body_name == 'left_ankle_roll_link':
                self.left_foot_geom_ids.append(i)
            elif body_name == 'right_ankle_roll_link':
                self.right_foot_geom_ids.append(i)

        self.joint_names = [
            'left_hip_pitch_joint', 'left_hip_roll_joint', 'left_hip_yaw_joint',
            'left_knee_joint', 'left_ankle_pitch_joint', 'left_ankle_roll_joint',
            'right_hip_pitch_joint', 'right_hip_roll_joint', 'right_hip_yaw_joint',
            'right_knee_joint', 'right_ankle_pitch_joint', 'right_ankle_roll_joint'
        ]

        self.sim_thread = threading.Thread(target=self.run_sim_loop, daemon=True)
        self.sim_thread.start()

    # ---------- callback ROS (thread di spin) ----------

    def cmd_vel_callback(self, msg: TwistStamped):
        self.cmd[0] = msg.twist.linear.x
        self.cmd[1] = msg.twist.linear.y
        self.cmd[2] = msg.twist.angular.z
        self.last_cmd_rx = time.monotonic()
        latency_ns = (self.get_clock().now() - rclpy.time.Time.from_msg(msg.header.stamp)).nanoseconds
        self.latency_pub.publish(Float64(data=latency_ns / 1e6))

    def rec_status_callback(self, msg: String):
        self.pending_rec = msg.data  # gestito dal sim thread

    def reset_callback(self, msg: Empty):
        self.reset_requested = True

    # ---------- helpers eseguiti SOLO nel sim thread ----------

    def _do_reset(self):
        mujoco.mj_resetData(self.m, self.d)
        self.counter = 0
        self.action[:] = 0.0
        self.obs[:] = 0.0
        self.target_dof_pos[:] = self.default_angles
        self.cmd[:] = 0.0
        self.reset_requested = False
        self.get_logger().info("Simulation reset to deterministic initial state.")

    def _handle_pending_recording(self):
        pending, self.pending_rec = self.pending_rec, None
        if pending is None:
            return
        if pending == "":
            with self.csv_lock:
                if self.csv_file:
                    self.is_logging_csv = False
                    self.csv_file.close()
                    self.csv_file = None
            self.get_logger().info("Closed CSV telemetry log.")
            return
        # Start: reset deterministico PRIMA di aprire il log
        self._do_reset()
        os.makedirs(self.bag_dir, exist_ok=True)
        csv_path = os.path.join(self.bag_dir, f"{pending}.csv")
        try:
            with self.csv_lock:
                self.csv_file = open(csv_path, 'w')
                qpos_cols = ",".join(f"qpos_{i}" for i in range(self.m.nq))
                qvel_cols = ",".join(f"qvel_{i}" for i in range(self.m.nv))
                target_cols = ",".join(f"target_{i}" for i in range(self.num_actions))
                self.csv_file.write(f"sim_time,pos_x,pos_y,pos_z,cmd_vx,cmd_vy,cmd_wz,{qpos_cols},{qvel_cols},{target_cols}\n")
                self.is_logging_csv = True
            self._write_csv_row()  # riga 0: stato iniziale post-reset (sim_time = 0)
            self.get_logger().info(f"Opened CSV telemetry log: {csv_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to open CSV log file: {e}")

    def _write_csv_row(self):
        with self.csv_lock:
            if not self.csv_file:
                return
            sim_time = self.counter * self.simulation_dt
            qpos_str = ",".join(str(x) for x in self.d.qpos)
            qvel_str = ",".join(str(x) for x in self.d.qvel)
            target_str = ",".join(str(x) for x in self.target_dof_pos)
            self.csv_file.write(
                f"{sim_time},{self.d.qpos[0]},{self.d.qpos[1]},{self.d.qpos[2]},"
                f"{self.cmd[0]},{self.cmd[1]},{self.cmd[2]},{qpos_str},{qvel_str},{target_str}\n")

    # ---------- loop di simulazione ----------

    def run_sim_loop(self):
        viewer = None
        if not self.headless:
            try:
                viewer = mujoco.viewer.launch_passive(self.m, self.d)
                self.get_logger().info("MuJoCo passive viewer launched.")
            except Exception as e:
                self.get_logger().error(f"Viewer failed: {e}. Falling back to headless.")
                viewer = None

        try:
            while rclpy.ok() and (viewer is None or viewer.is_running()):
                step_start = time.time()

                self._handle_pending_recording()
                if self.reset_requested:
                    self._do_reset()

                if self.cmd.any() and (time.monotonic() - self.last_cmd_rx) > self.cmd_timeout:
                    self.cmd[:] = 0.0
                    self.get_logger().warn("cmd_vel timeout: command zeroed for safety",
                                           throttle_duration_sec=5.0)

                tau = sim_utils.pd_control(
                    self.target_dof_pos, self.d.qpos[7:], self.kps,
                    np.zeros_like(self.kds), self.d.qvel[6:], self.kds)
                self.d.ctrl[:] = tau
                mujoco.mj_step(self.m, self.d)

                self.counter += 1
                if self.counter % self.control_decimation == 0:
                    self.evaluate_policy()
                self.publish_telemetries()

                if viewer is not None and self.counter % self.control_decimation == 0:
                    viewer.sync()

                time_until_next_step = self.simulation_dt - (time.time() - step_start)
                if time_until_next_step > 0:
                    time.sleep(time_until_next_step)
        finally:
            if viewer is not None:
                viewer.close()

    def evaluate_policy(self):
        qj = (self.d.qpos[7:] - self.default_angles) * self.dof_pos_scale
        dqj = self.d.qvel[6:] * self.dof_vel_scale
        gravity_orientation = sim_utils.get_gravity_orientation(self.d.qpos[3:7])
        omega = self.d.qvel[3:6] * self.ang_vel_scale

        period = 0.8
        phase = (self.counter * self.simulation_dt) % period / period
        sin_phase = np.sin(2 * np.pi * phase)
        cos_phase = np.cos(2 * np.pi * phase)

        n = self.num_actions
        self.obs[:3] = omega
        self.obs[3:6] = gravity_orientation
        self.obs[6:9] = self.cmd * self.cmd_scale
        self.obs[9:9 + n] = qj
        self.obs[9 + n:9 + 2 * n] = dqj
        self.obs[9 + 2 * n:9 + 3 * n] = self.action
        self.obs[9 + 3 * n:9 + 3 * n + 2] = np.array([sin_phase, cos_phase])

        obs_tensor = torch.from_numpy(self.obs).unsqueeze(0)
        with torch.no_grad():
            self.action[:] = self.policy(obs_tensor).numpy().squeeze()
        self.target_dof_pos[:] = self.action * self.action_scale + self.default_angles


    def publish_telemetries(self):
        if self.is_logging_csv:
            self._write_csv_row()

        # 1. Contatti piedi
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
        self.left_contact_pub.publish(Bool(data=left_contact))
        self.right_contact_pub.publish(Bool(data=right_contact))

        fallen = sim_utils.is_fallen(self.d.qpos[3], self.d.qpos[4],
                                     self.d.qpos[5], self.d.qpos[6],
                                     self.fall_threshold_deg)
        self.fall_pub.publish(Bool(data=fallen))

        now = self.get_clock().now().to_msg()

        # 2. IMU
        imu_msg = Imu()
        imu_msg.header.stamp = now
        imu_msg.header.frame_id = "pelvis"
        imu_msg.orientation.w = float(self.d.qpos[3])
        imu_msg.orientation.x = float(self.d.qpos[4])
        imu_msg.orientation.y = float(self.d.qpos[5])
        imu_msg.orientation.z = float(self.d.qpos[6])
        imu_msg.angular_velocity.x = float(self.d.qvel[3])
        imu_msg.angular_velocity.y = float(self.d.qvel[4])
        imu_msg.angular_velocity.z = float(self.d.qvel[5])
        R = self.d.xmat[self.pelvis_body_id].reshape(3, 3)
        acc_world = self.d.qacc[0:3] + np.array([0.0, 0.0, 9.81])
        acc_body = R.T @ acc_world
        imu_msg.linear_acceleration.x = float(acc_body[0])
        imu_msg.linear_acceleration.y = float(acc_body[1])
        imu_msg.linear_acceleration.z = float(acc_body[2])
        self.imu_pub.publish(imu_msg)

        # 3. Joint states
        js_msg = JointState()
        js_msg.header.stamp = now
        js_msg.name = self.joint_names
        js_msg.position = self.d.qpos[7:].tolist()
        js_msg.velocity = self.d.qvel[6:].tolist()
        js_msg.effort = self.d.ctrl[:].tolist()
        self.joint_pub.publish(js_msg)

        # 4. Odometria
        odom_msg = Odometry()
        odom_msg.header.stamp = now
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "pelvis"
        odom_msg.pose.pose.position.x = float(self.d.qpos[0])
        odom_msg.pose.pose.position.y = float(self.d.qpos[1])
        odom_msg.pose.pose.position.z = float(self.d.qpos[2])
        odom_msg.pose.pose.orientation.w = float(self.d.qpos[3])
        odom_msg.pose.pose.orientation.x = float(self.d.qpos[4])
        odom_msg.pose.pose.orientation.y = float(self.d.qpos[5])
        odom_msg.pose.pose.orientation.z = float(self.d.qpos[6])
        vel_body = R.T @ self.d.qvel[0:3]
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
