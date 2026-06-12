#!/usr/bin/env python3
import os
import sys
import copy
import time
import subprocess
import threading
import signal
from flask import Flask, render_template, jsonify, request
from flask_socketio import SocketIO, emit
import pygame

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Bool, String, Empty, Float64
from nav_msgs.msg import Odometry

from ament_index_python.packages import get_package_share_directory

_share = get_package_share_directory('progetto_robotica')
app = Flask(
    __name__,
    template_folder=os.path.join(_share, 'web', 'templates'),
    static_folder=os.path.join(_share, 'web', 'static'),
)
socketio = SocketIO(app, cors_allowed_origins="*")

# Shared telemetry state
telemetry_state = {
    'imu': {
        'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0},
        'angular_velocity': {'x': 0.0, 'y': 0.0, 'z': 0.0},
        'linear_acceleration': {'x': 0.0, 'y': 0.0, 'z': 0.0}
    },
    'joints': {
        'names': [],
        'position': [],
        'velocity': [],
        'effort': []
    },
    'contacts': {
        'left': False,
        'right': False
    },
    'odom': {
        'position': {'x': 0.0, 'y': 0.0, 'z': 0.0},
        'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0},
        'linear_velocity': {'x': 0.0, 'y': 0.0, 'z': 0.0},
        'angular_velocity': {'x': 0.0, 'y': 0.0, 'z': 0.0}
    },
    'latency_ms': 0.0,
    'fall_detected': False,
    'last_update': 0.0
}

state_lock = threading.Lock()

class WebTeleopNode(Node):
    def __init__(self):
        super().__init__('web_teleop_node')
        
        self.declare_parameter('bag_dir', os.path.expanduser('~/progetto_robotica_bags'))
        self.bag_dir = os.path.expanduser(self.get_parameter('bag_dir').value)
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.rec_status_pub = self.create_publisher(String, '/recording_status', 10)
        self.reset_pub = self.create_publisher(Empty, '/sim_reset', 10)
        
        # Subscribers for telemetry
        self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.create_subscription(JointState, '/joint_states', self.joint_states_callback, 10)
        self.create_subscription(Bool, '/contacts/left', self.left_contact_callback, 10)
        self.create_subscription(Bool, '/contacts/right', self.right_contact_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(Float64, '/metrics/cmd_latency_ms', self.latency_callback, 10)
        self.create_subscription(Bool, '/fall_detected', self.fall_callback, 10)
        
        self.record_process = None
        self.bag_name = ""
        self.is_recording = False
        
        # Telemetry streaming thread
        self.stream_thread = threading.Thread(target=self.stream_telemetry_loop)
        self.stream_thread.daemon = True
        self.stream_thread.start()
        
        # Gamepad teleop thread
        self.gamepad_thread = threading.Thread(target=self.gamepad_loop)
        self.gamepad_thread.daemon = True
        self.gamepad_thread.start()

    def imu_callback(self, msg: Imu):
        with state_lock:
            telemetry_state['imu']['orientation'] = {
                'x': msg.orientation.x,
                'y': msg.orientation.y,
                'z': msg.orientation.z,
                'w': msg.orientation.w
            }
            telemetry_state['imu']['angular_velocity'] = {
                'x': msg.angular_velocity.x,
                'y': msg.angular_velocity.y,
                'z': msg.angular_velocity.z
            }
            telemetry_state['imu']['linear_acceleration'] = {
                'x': msg.linear_acceleration.x,
                'y': msg.linear_acceleration.y,
                'z': msg.linear_acceleration.z
            }
            telemetry_state['last_update'] = time.time()

    def joint_states_callback(self, msg: JointState):
        with state_lock:
            telemetry_state['joints']['names'] = list(msg.name)
            telemetry_state['joints']['position'] = [float(x) for x in msg.position]
            telemetry_state['joints']['velocity'] = [float(x) for x in msg.velocity]
            telemetry_state['joints']['effort'] = [float(x) for x in msg.effort]
            telemetry_state['last_update'] = time.time()

    def left_contact_callback(self, msg: Bool):
        with state_lock:
            telemetry_state['contacts']['left'] = msg.data
            telemetry_state['last_update'] = time.time()

    def right_contact_callback(self, msg: Bool):
        with state_lock:
            telemetry_state['contacts']['right'] = msg.data
            telemetry_state['last_update'] = time.time()

    def odom_callback(self, msg: Odometry):
        with state_lock:
            telemetry_state['odom']['position'] = {
                'x': msg.pose.pose.position.x,
                'y': msg.pose.pose.position.y,
                'z': msg.pose.pose.position.z
            }
            telemetry_state['odom']['orientation'] = {
                'x': msg.pose.pose.orientation.x,
                'y': msg.pose.pose.orientation.y,
                'z': msg.pose.pose.orientation.z,
                'w': msg.pose.pose.orientation.w
            }
            telemetry_state['odom']['linear_velocity'] = {
                'x': msg.twist.twist.linear.x,
                'y': msg.twist.twist.linear.y,
                'z': msg.twist.twist.linear.z
            }
            telemetry_state['odom']['angular_velocity'] = {
                'x': msg.twist.twist.angular.x,
                'y': msg.twist.twist.angular.y,
                'z': msg.twist.twist.angular.z
            }
            telemetry_state['last_update'] = time.time()

    def latency_callback(self, msg: Float64):
        with state_lock:
            telemetry_state['latency_ms'] = round(msg.data, 1)

    def fall_callback(self, msg: Bool):
        with state_lock:
            telemetry_state['fall_detected'] = msg.data

    def publish_cmd_vel(self, vx: float, vy: float, wz: float):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = float(vx)
        msg.twist.linear.y = float(vy)
        msg.twist.angular.z = float(wz)
        self.cmd_vel_pub.publish(msg)

    def stream_telemetry_loop(self):
        # Scheduling compensato: il periodo non accumula il tempo di emit/serializzazione
        period = 1.0 / 33.0  # target 33 Hz per garantire >= 30 Hz effettivi
        next_t = time.monotonic()
        while rclpy.ok():
            with state_lock:
                snapshot = copy.deepcopy(telemetry_state)
            socketio.emit('telemetry', snapshot)
            next_t += period
            sleep_t = next_t - time.monotonic()
            if sleep_t > 0:
                time.sleep(sleep_t)
            else:
                next_t = time.monotonic()  # in ritardo: riallinea senza burst

    def gamepad_loop(self):
        # Configure headless env for pygame if window is not needed
        os.environ["SDL_VIDEODRIVER"] = "dummy"
        pygame.init()
        pygame.joystick.init()
        
        self.get_logger().info("Starting gamepad monitoring loop...")
        joystick = None
        
        while rclpy.ok():
            pygame.event.pump()
            joy_count = pygame.joystick.get_count()
            
            if joy_count > 0 and joystick is None:
                joystick = pygame.joystick.Joystick(0)
                joystick.init()
                self.get_logger().info(f"Gamepad connected: {joystick.get_name()}")
                
            if joystick is not None:
                try:
                    # Map axes (adjust based on standard Xbox/DualShock layout)
                    # Axis 1: Left stick Y (forward/backward) -> negative is forward in pygame
                    # Axis 0: Left stick X (lateral)
                    # Axis 3: Right stick X (yaw rate)
                    vx = -joystick.get_axis(1) * 0.8  # Max speed 0.8 m/s
                    vy = -joystick.get_axis(0) * 0.4  # Max lateral speed 0.4 m/s
                    wz = -joystick.get_axis(3) * 0.8  # Max yaw rate 0.8 rad/s
                    
                    # Apply deadzones
                    if abs(vx) < 0.1: vx = 0.0
                    if abs(vy) < 0.1: vy = 0.0
                    if abs(wz) < 0.1: wz = 0.0
                    
                    self.publish_cmd_vel(vx, vy, wz)
                except pygame.error:
                    self.get_logger().warn("Gamepad read error. Resetting connection.")
                    joystick = None
            
            time.sleep(0.02) # Read gamepad at 50Hz

    def start_recording(self):
        if self.is_recording:
            return False, "Already recording"
            
        bag_dir = self.bag_dir
        os.makedirs(bag_dir, exist_ok=True)
        
        self.bag_name = f"bag_{time.strftime('%Y%m%d_%H%M%S')}"
        bag_path = os.path.join(bag_dir, self.bag_name)
        
        self.get_logger().info(f"Starting rosbag2 recording to: {bag_path}")
        
        cmd = [
            "ros2", "bag", "record",
            "-o", bag_path,
            "/cmd_vel",
            "/imu",
            "/joint_states",
            "/contacts/left",
            "/contacts/right",
            "/odom",
            "/fall_detected",
            "/metrics/cmd_latency_ms"
        ]
        
        self.record_process = subprocess.Popen(
            cmd,
            preexec_fn=os.setsid,  # Start in a new process group to catch child processes on exit
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )
        self.is_recording = True
        
        # Publish status to simulator to start CSV logging
        status_msg = String()
        status_msg.data = self.bag_name
        self.rec_status_pub.publish(status_msg)
        
        return True, self.bag_name

    def stop_recording(self):
        if not self.is_recording or self.record_process is None:
            return False, "Not recording"
            
        self.get_logger().info("Stopping rosbag2 recording...")
        try:
            # Send SIGINT to the process group to ensure ros2 bag record finishes gracefully
            os.killpg(os.getpgid(self.record_process.pid), signal.SIGINT)
            self.record_process.wait(timeout=5.0)
        except Exception as e:
            self.get_logger().error(f"Error terminating rosbag2 process: {e}")
            self.record_process.kill()
            
        self.record_process = None
        self.is_recording = False
        
        # Publish empty status to simulator to close CSV logging
        status_msg = String()
        status_msg.data = ""
        self.rec_status_pub.publish(status_msg)
        
        return True, self.bag_name

# Global node reference
ros_node = None

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/api/status')
def get_status():
    return jsonify({
        'is_recording': ros_node.is_recording,
        'bag_name': ros_node.bag_name,
        'telemetry_age_seconds': time.time() - telemetry_state['last_update']
    })

@app.route('/api/record/start', methods=['POST'])
def handle_start_record():
    success, info = ros_node.start_recording()
    return jsonify({'success': success, 'info': info})

@app.route('/api/record/stop', methods=['POST'])
def handle_stop_record():
    success, info = ros_node.stop_recording()
    return jsonify({'success': success, 'info': info})

@app.route('/api/reset', methods=['POST'])
def handle_reset():
    ros_node.reset_pub.publish(Empty())
    return jsonify({'success': True})

@socketio.on('teleop_cmd')
def handle_teleop_cmd(data):
    # Retrieve speed from payload
    vx = data.get('vx', 0.0)
    vy = data.get('vy', 0.0)
    wz = data.get('wz', 0.0)
    ros_node.publish_cmd_vel(vx, vy, wz)

@socketio.on('latency_ping')
def handle_latency_ping(data):
    emit('latency_pong', data)

def main(args=None):
    global ros_node
    rclpy.init(args=args)
    ros_node = WebTeleopNode()
    
    # Spin ROS 2 in a background thread
    ros_thread = threading.Thread(target=lambda: rclpy.spin(ros_node))
    ros_thread.daemon = True
    ros_thread.start()
    
    # Run the Flask-SocketIO server in the main thread
    # The default port forwarding mapping from WSL to Windows localhost works automatically
    try:
        ros_node.get_logger().info("Starting Flask-SocketIO server on port 5000...")
        socketio.run(app, host='0.0.0.0', port=5000, allow_unsafe_werkzeug=True)
    except KeyboardInterrupt:
        pass
    finally:
        if ros_node.is_recording:
            ros_node.stop_recording()
        ros_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
