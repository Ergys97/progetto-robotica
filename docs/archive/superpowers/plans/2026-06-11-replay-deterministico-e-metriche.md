# Replay Deterministico, Path Riproducibili e Strumentazione Metriche — Piano di Implementazione

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Rendere il prototipo G1 teleop conforme ai deliverable della Traccia 10: replay con MSE ≈ 0, deployment riproducibile senza path hardcoded, metriche (telemetria ≥30 Hz, latenza <50 ms, fall/flight detection) misurabili da backend.

**Architecture:** Si mantiene l'architettura a due nodi (mujoco_sim = bridge MuJoCo+policy RL, web_teleop = Flask-SocketIO+teleop). Si estrae la logica pura in `sim_utils.py` (testabile con pytest senza ROS), si sposta tutto lo stato di simulazione in attributi del nodo per permettere un reset deterministico, e il logging CSV passa da wall-clock a sim-time.

**Tech Stack:** ROS 2 Jazzy (WSL Ubuntu 24.04), MuJoCo Python bindings, PyTorch (policy JIT unitree_rl_gym), Flask-SocketIO, pytest.

---

## Prerequisiti ambiente (da fare una volta, prima del Task 1)

Tutti i comandi di build/test vanno eseguiti **dentro WSL**, non su /mnt/c (filesystem 9P lento, causa drop nei log ad alta frequenza).

1. In WSL deve esistere `~/unitree_rl_gym` (già presente: i path attuali lo referenziano).
2. ROS 2 Jazzy installato e aggiornato (`sudo apt update && sudo apt upgrade`).
3. Python deps nel venv `~/venv` (creato con `--system-site-packages` così vede rclpy):
   `~/venv/bin/pip install mujoco torch flask flask-socketio pygame numpy pyyaml pytest`
4. Workspace ROS in WSL: `mkdir -p ~/ros2_ws/src`. Dopo il Task 1 (git init + push), clonare lì:
   `git clone <remote-url> ~/ros2_ws/src/progetto_robotica`. **Da quel momento si lavora nel clone WSL** e la cartella Windows si aggiorna via `git pull`.
5. Build: `cd ~/ros2_ws && source /opt/ros/jazzy/setup.bash && colcon build --symlink-install && source install/setup.bash`

---

### Task 1: Git init, .gitignore, commit baseline

**Files:**
- Create: `.gitignore`
- (repo: `git init` nella cartella corrente)

- [ ] **Step 1: Crea `.gitignore`**

```gitignore
__pycache__/
*.pyc
bags/
build/
install/
log/
*.egg-info/
.pytest_cache/
```

- [ ] **Step 2: Inizializza il repo e commit baseline**

```bash
git init
git add .
git commit -m "chore: baseline progetto teleop G1 (pre-refactor)"
```

- [ ] **Step 3: (manuale/facoltativo ora) crea il repo GitHub e push**

```bash
git remote add origin <url>
git push -u origin main
```

---

### Task 2: Estrarre la logica pura in `sim_utils.py` (TDD)

**Files:**
- Create: `progetto_robotica/sim_utils.py`
- Create: `test/test_sim_utils.py`
- Modify: `progetto_robotica/mujoco_sim.py` (usa sim_utils al posto dei metodi inline)

- [ ] **Step 1: Scrivi i test (falliranno)**

`test/test_sim_utils.py`:

```python
import numpy as np
import pytest

from progetto_robotica.sim_utils import (
    get_gravity_orientation, pd_control, quat_to_roll_pitch,
    is_fallen, find_log_index,
)


def test_gravity_identity():
    # robot dritto: gravità proiettata = [0, 0, -1]
    g = get_gravity_orientation(np.array([1.0, 0.0, 0.0, 0.0]))
    np.testing.assert_allclose(g, [0.0, 0.0, -1.0], atol=1e-9)


def test_gravity_roll_90():
    c = np.cos(np.pi / 4)
    s = np.sin(np.pi / 4)
    g = get_gravity_orientation(np.array([c, s, 0.0, 0.0]))  # roll +90°
    np.testing.assert_allclose(g, [0.0, -1.0, 0.0], atol=1e-9)


def test_pd_control():
    tau = pd_control(np.array([1.0]), np.array([0.0]), np.array([2.0]),
                     np.array([0.0]), np.array([0.5]), np.array([4.0]))
    np.testing.assert_allclose(tau, [2.0 - 2.0], atol=1e-9)  # 2*(1-0) + 4*(0-0.5)


def test_roll_pitch_identity():
    roll, pitch = quat_to_roll_pitch(1.0, 0.0, 0.0, 0.0)
    assert abs(roll) < 1e-9 and abs(pitch) < 1e-9


def test_is_fallen_threshold():
    # roll = 40° > soglia 35° -> caduto
    half = np.deg2rad(40.0) / 2
    assert is_fallen(np.cos(half), np.sin(half), 0.0, 0.0, 35.0) is True
    # roll = 30° < soglia -> ok
    half = np.deg2rad(30.0) / 2
    assert is_fallen(np.cos(half), np.sin(half), 0.0, 0.0, 35.0) is False


def test_find_log_index():
    times = np.array([0.0, 0.02, 0.04])
    assert find_log_index(times, 0.0) == 0
    assert find_log_index(times, 0.019) == 0
    assert find_log_index(times, 0.02) == 1
    assert find_log_index(times, 99.0) == 2
    assert find_log_index(times, -1.0) == 0
```

- [ ] **Step 2: Esegui i test e verifica che falliscano**

Run (in WSL, dalla root del pacchetto): `~/venv/bin/python -m pytest test/ -v`
Expected: FAIL con `ModuleNotFoundError: No module named 'progetto_robotica.sim_utils'`

- [ ] **Step 3: Implementa `progetto_robotica/sim_utils.py`**

```python
"""Pure-math helpers shared by mujoco_sim and replay_eval (no ROS deps)."""
import numpy as np


def get_gravity_orientation(quaternion):
    """Gravity vector projected in body frame. quaternion = [qw, qx, qy, qz]."""
    qw, qx, qy, qz = quaternion[0], quaternion[1], quaternion[2], quaternion[3]
    g = np.zeros(3)
    g[0] = 2 * (-qz * qx + qw * qy)
    g[1] = -2 * (qz * qy + qw * qx)
    g[2] = 1 - 2 * (qw * qw + qz * qz)
    return g


def pd_control(target_q, q, kp, target_dq, dq, kd):
    return (target_q - q) * kp + (target_dq - dq) * kd


def quat_to_roll_pitch(qw, qx, qy, qz):
    sinr_cosp = 2 * (qw * qx + qy * qz)
    cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
    roll = np.arctan2(sinr_cosp, cosr_cosp)
    sinp = 2 * (qw * qy - qz * qx)
    pitch = np.arcsin(np.clip(sinp, -1.0, 1.0))
    return roll, pitch


def is_fallen(qw, qx, qy, qz, threshold_deg):
    roll, pitch = quat_to_roll_pitch(qw, qx, qy, qz)
    t = np.deg2rad(threshold_deg)
    return bool(abs(roll) > t or abs(pitch) > t)


def find_log_index(sim_times, t):
    """Index of the last logged row with sim_time <= t (clamped to valid range)."""
    i = int(np.searchsorted(sim_times, t, side='right')) - 1
    return max(0, min(i, len(sim_times) - 1))
```

- [ ] **Step 4: Esegui i test e verifica che passino**

Run: `~/venv/bin/python -m pytest test/ -v`
Expected: 6 passed

- [ ] **Step 5: Usa sim_utils in mujoco_sim.py**

In `progetto_robotica/mujoco_sim.py`:
- aggiungi `from progetto_robotica import sim_utils` dopo gli import ROS;
- elimina i metodi `get_gravity_orientation` e `pd_control` dalla classe;
- in `run_sim_loop` sostituisci entrambe le occorrenze di `self.pd_control(...)` con `sim_utils.pd_control(...)`;
- in `evaluate_policy` sostituisci `self.get_gravity_orientation(quat)` con `sim_utils.get_gravity_orientation(quat)`.

- [ ] **Step 6: Commit**

```bash
git add progetto_robotica/sim_utils.py test/test_sim_utils.py progetto_robotica/mujoco_sim.py
git commit -m "refactor: extract pure sim helpers into sim_utils with pytest coverage"
```

---

### Task 3: Packaging e path riproducibili

**Files:**
- Modify: `setup.py`
- Modify: `package.xml`
- Create: `requirements.txt`
- Modify: `progetto_robotica/web_teleop.py` (share dir + param bag_dir)
- Modify: `launch/teleop_sim_launch.py`

- [ ] **Step 1: Riscrivi `setup.py`**

```python
from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'progetto_robotica'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*_launch.py')),
        (os.path.join('share', package_name, 'web', 'templates'), glob('web/templates/*.html')),
        (os.path.join('share', package_name, 'web', 'static', 'css'), glob('web/static/css/*.css')),
        (os.path.join('share', package_name, 'web', 'static', 'js'), glob('web/static/js/*.js')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ergys',
    maintainer_email='ergysperdeda97@gmail.com',
    description='ROS 2 Teleoperation and Web Dashboard for Unitree G1 in MuJoCo',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mujoco_sim = progetto_robotica.mujoco_sim:main',
            'web_teleop = progetto_robotica.web_teleop:main',
            'replay_eval = progetto_robotica.replay_eval:main',
        ],
    },
)
```

(Nota: `replay_eval:main` viene creato nel Task 5; fino ad allora l'entry point esiste ma non va invocato.)

- [ ] **Step 2: Aggiorna `package.xml`** (aggiungi dopo `<depend>nav_msgs</depend>`)

```xml
  <exec_depend>ament_index_python</exec_depend>
  <exec_depend>ros2bag</exec_depend>
  <exec_depend>rosbag2_storage_mcap</exec_depend>
```

- [ ] **Step 3: Crea `requirements.txt`**

```text
mujoco>=3.0
torch
numpy
pyyaml
flask
flask-socketio
pygame
pytest
```

- [ ] **Step 4: web_teleop.py — Flask dalle share dir, bag_dir come parametro**

Sostituisci la creazione dell'app (righe 19–24) con:

```python
from ament_index_python.packages import get_package_share_directory

_share = get_package_share_directory('progetto_robotica')
app = Flask(
    __name__,
    template_folder=os.path.join(_share, 'web', 'templates'),
    static_folder=os.path.join(_share, 'web', 'static'),
)
socketio = SocketIO(app, cors_allowed_origins="*")
```

In `WebTeleopNode.__init__`, subito dopo `super().__init__(...)`:

```python
        self.declare_parameter('bag_dir', os.path.expanduser('~/progetto_robotica_bags'))
        self.bag_dir = os.path.expanduser(self.get_parameter('bag_dir').value)
```

In `start_recording` sostituisci la riga `bag_dir = "/mnt/c/..."` con `bag_dir = self.bag_dir`.

- [ ] **Step 5: mujoco_sim.py — default da env var, bag_dir parametro**

In testa al file (dopo gli import):

```python
DEFAULT_RL_GYM = os.path.expanduser(os.environ.get('UNITREE_RL_GYM_PATH', '~/unitree_rl_gym'))
```

Sostituisci i 4 `declare_parameter` con:

```python
        self.declare_parameter('xml_path', os.path.join(DEFAULT_RL_GYM, 'resources/robots/g1_description/scene.xml'))
        self.declare_parameter('policy_path', os.path.join(DEFAULT_RL_GYM, 'deploy/pre_train/g1/motion.pt'))
        self.declare_parameter('config_path', os.path.join(DEFAULT_RL_GYM, 'deploy/deploy_mujoco/configs/g1.yaml'))
        self.declare_parameter('headless', False)
        self.declare_parameter('bag_dir', os.path.expanduser('~/progetto_robotica_bags'))
        self.bag_dir = os.path.expanduser(self.get_parameter('bag_dir').value)
```

In `rec_status_callback` sostituisci `bag_dir = "/mnt/c/..."` con `bag_dir = self.bag_dir`.

- [ ] **Step 6: Riscrivi `launch/teleop_sim_launch.py`**

```python
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

RL_GYM = os.path.expanduser(os.environ.get('UNITREE_RL_GYM_PATH', '~/unitree_rl_gym'))


def generate_launch_description():
    args = [
        DeclareLaunchArgument('headless', default_value='False'),
        DeclareLaunchArgument('xml_path',
            default_value=os.path.join(RL_GYM, 'resources/robots/g1_description/scene.xml')),
        DeclareLaunchArgument('policy_path',
            default_value=os.path.join(RL_GYM, 'deploy/pre_train/g1/motion.pt')),
        DeclareLaunchArgument('config_path',
            default_value=os.path.join(RL_GYM, 'deploy/deploy_mujoco/configs/g1.yaml')),
        DeclareLaunchArgument('bag_dir',
            default_value=os.path.expanduser('~/progetto_robotica_bags')),
    ]

    sim_node = Node(
        package='progetto_robotica', executable='mujoco_sim',
        name='mujoco_sim_node', output='screen',
        parameters=[{
            'headless': LaunchConfiguration('headless'),
            'xml_path': LaunchConfiguration('xml_path'),
            'policy_path': LaunchConfiguration('policy_path'),
            'config_path': LaunchConfiguration('config_path'),
            'bag_dir': LaunchConfiguration('bag_dir'),
        }],
    )

    web_node = Node(
        package='progetto_robotica', executable='web_teleop',
        name='web_teleop_node', output='screen',
        parameters=[{'bag_dir': LaunchConfiguration('bag_dir')}],
    )

    return LaunchDescription(args + [sim_node, web_node])
```

- [ ] **Step 7: Build e verifica installazione asset**

```bash
cd ~/ros2_ws && colcon build --symlink-install && source install/setup.bash
ls install/progetto_robotica/share/progetto_robotica/web/templates/
```
Expected: `index.html`

- [ ] **Step 8: Verifica grep — nessun path hardcoded residuo**

Run: `grep -rn "/mnt/c\|/home/ergys" progetto_robotica/ launch/ --include="*.py"`
Expected: nessun risultato (replay_eval.py verrà sistemato nel Task 5; se compare solo lì è OK per ora).

- [ ] **Step 9: Commit**

```bash
git add setup.py package.xml requirements.txt progetto_robotica/web_teleop.py progetto_robotica/mujoco_sim.py launch/teleop_sim_launch.py
git commit -m "fix: remove hardcoded paths, install web assets via share dir, declare deps"
```

---

### Task 4: Refactor stato simulazione + reset deterministico + CSV in sim-time

**Files:**
- Modify: `progetto_robotica/mujoco_sim.py` (riscrittura: stato in attributi, reset, CSV sim-time, pelvis id, loop unificato)
- Modify: `progetto_robotica/web_teleop.py` (route `/api/reset` → publish su `/sim_reset`)
- Modify: `web/templates/index.html` + `web/static/js/dashboard.js` (bottone Reset)

Obiettivo: la registrazione parte SEMPRE da uno stato resettato (mj_resetData, counter=0, action=0, fase gait=0) e il CSV logga `sim_time = counter*dt`. Così live e replay condividono identico stato iniziale e identica timeline ⇒ MSE ≈ 0.

- [ ] **Step 1: Riscrivi `progetto_robotica/mujoco_sim.py`**

```python
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
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Bool, Empty, String
from nav_msgs.msg import Odometry

from progetto_robotica import sim_utils

DEFAULT_RL_GYM = os.path.expanduser(os.environ.get('UNITREE_RL_GYM_PATH', '~/unitree_rl_gym'))


class MuJoCoSimNode(Node):
    def __init__(self):
        super().__init__('mujoco_sim_node')

        self.declare_parameter('xml_path', os.path.join(DEFAULT_RL_GYM, 'resources/robots/g1_description/scene.xml'))
        self.declare_parameter('policy_path', os.path.join(DEFAULT_RL_GYM, 'deploy/pre_train/g1/motion.pt'))
        self.declare_parameter('config_path', os.path.join(DEFAULT_RL_GYM, 'deploy/deploy_mujoco/configs/g1.yaml'))
        self.declare_parameter('headless', False)
        self.declare_parameter('bag_dir', os.path.expanduser('~/progetto_robotica_bags'))

        xml_path = self.get_parameter('xml_path').value
        policy_path = self.get_parameter('policy_path').value
        config_path = self.get_parameter('config_path').value
        self.headless = self.get_parameter('headless').value
        self.bag_dir = os.path.expanduser(self.get_parameter('bag_dir').value)

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

        # ROS I/O
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.rec_status_sub = self.create_subscription(String, '/recording_status', self.rec_status_callback, 10)
        self.reset_sub = self.create_subscription(Empty, '/sim_reset', self.reset_callback, 10)
        self.imu_pub = self.create_publisher(Imu, '/imu', 10)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.left_contact_pub = self.create_publisher(Bool, '/contacts/left', 10)
        self.right_contact_pub = self.create_publisher(Bool, '/contacts/right', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # MuJoCo
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

    def cmd_vel_callback(self, msg: Twist):
        self.cmd[0] = msg.linear.x
        self.cmd[1] = msg.linear.y
        self.cmd[2] = msg.angular.z

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
                self.csv_file.write(f"sim_time,pos_x,pos_y,pos_z,cmd_vx,cmd_vy,cmd_wz,{qpos_cols},{qvel_cols}\n")
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
            self.csv_file.write(
                f"{sim_time},{self.d.qpos[0]},{self.d.qpos[1]},{self.d.qpos[2]},"
                f"{self.cmd[0]},{self.cmd[1]},{self.cmd[2]},{qpos_str},{qvel_str}\n")

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

                tau = sim_utils.pd_control(
                    self.target_dof_pos, self.d.qpos[7:], self.kps,
                    np.zeros_like(self.kds), self.d.qvel[6:], self.kds)
                self.d.ctrl[:] = tau
                mujoco.mj_step(self.m, self.d)

                self.counter += 1
                if self.counter % self.control_decimation == 0:
                    self.evaluate_policy()
                    self.publish_telemetries()

                if viewer is not None:
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
```

- [ ] **Step 2: web_teleop.py — publisher `/sim_reset` e route `/api/reset`**

In `WebTeleopNode.__init__`, dopo `self.rec_status_pub`:

```python
        self.reset_pub = self.create_publisher(Empty, '/sim_reset', 10)
```

Aggiungi `Empty` all'import: `from std_msgs.msg import Bool, String, Empty`.

Dopo la route `/api/record/stop` aggiungi:

```python
@app.route('/api/reset', methods=['POST'])
def handle_reset():
    ros_node.reset_pub.publish(Empty())
    return jsonify({'success': True})
```

- [ ] **Step 3: Bottone Reset nella dashboard**

In `web/templates/index.html`, dentro `<div class="btn-group">` dopo il bottone `btn-record-stop`:

```html
                                <button class="btn" id="btn-reset">
                                    <i class="fa-solid fa-rotate"></i> Reset Sim
                                </button>
```

In `web/static/js/dashboard.js`, in fondo al blocco "Recording & Session Management":

```javascript
document.getElementById('btn-reset').addEventListener('click', () => {
    fetch('/api/reset', { method: 'POST' });
});
```

- [ ] **Step 4: Verifica manuale**

```bash
cd ~/ros2_ws && colcon build --symlink-install && source install/setup.bash
ros2 launch progetto_robotica teleop_sim_launch.py headless:=True
```
Da un secondo terminale: apri http://localhost:5000, premi Start Record, cammina ~10 s con WASD, Stop Record. Poi:

```bash
head -3 ~/progetto_robotica_bags/bag_*.csv | cut -d',' -f1-7
```
Expected: prima colonna `sim_time` con valori `0.0`, `0.02`, `0.04`, … e prima riga con cmd tutti a 0 (stato post-reset).

- [ ] **Step 5: Commit**

```bash
git add progetto_robotica/mujoco_sim.py progetto_robotica/web_teleop.py web/templates/index.html web/static/js/dashboard.js
git commit -m "feat: deterministic reset on record start, sim-time CSV logging, /sim_reset topic"
```

---

### Task 5: Riscrittura `replay_eval.py` (replay deterministico + MSE)

**Files:**
- Rewrite: `progetto_robotica/replay_eval.py`

- [ ] **Step 1: Riscrivi il file**

```python
#!/usr/bin/env python3
"""Replay deterministico di una sessione registrata e calcolo MSE vs traiettoria live.

Uso: ros2 run progetto_robotica replay_eval <bag_name> [--bag-dir DIR] [--rl-gym DIR]
"""
import argparse
import csv
import os

import numpy as np
import torch
import yaml
import mujoco

from progetto_robotica import sim_utils

DEFAULT_RL_GYM = os.path.expanduser(os.environ.get('UNITREE_RL_GYM_PATH', '~/unitree_rl_gym'))
DEFAULT_BAG_DIR = os.path.expanduser('~/progetto_robotica_bags')


def run_replay(bag_name, bag_dir=DEFAULT_BAG_DIR, rl_gym=DEFAULT_RL_GYM):
    csv_path = os.path.join(bag_dir, f"{bag_name}.csv")
    if not os.path.exists(csv_path):
        print(f"Error: log file not found at {csv_path}")
        return False

    with open(csv_path, 'r') as f:
        reader = csv.DictReader(f)
        rows = list(reader)
    if not rows:
        print("Error: log file is empty.")
        return False

    sim_times = np.array([float(r['sim_time']) for r in rows])
    cmds = np.array([[float(r['cmd_vx']), float(r['cmd_vy']), float(r['cmd_wz'])] for r in rows],
                    dtype=np.float32)
    live_pos = np.array([[float(r['pos_x']), float(r['pos_y']), float(r['pos_z'])] for r in rows])
    print(f"Loaded {len(rows)} telemetry frames ({sim_times[-1]:.1f}s of sim time).")

    config_path = os.path.join(rl_gym, 'deploy/deploy_mujoco/configs/g1.yaml')
    xml_path = os.path.join(rl_gym, 'resources/robots/g1_description/scene.xml')
    policy_path = os.path.join(rl_gym, 'deploy/pre_train/g1/motion.pt')

    with open(config_path, 'r') as f:
        config = yaml.load(f, Loader=yaml.FullLoader)

    simulation_dt = config["simulation_dt"]
    control_decimation = config["control_decimation"]
    kps = np.array(config["kps"], dtype=np.float32)
    kds = np.array(config["kds"], dtype=np.float32)
    default_angles = np.array(config["default_angles"], dtype=np.float32)
    ang_vel_scale = config["ang_vel_scale"]
    dof_pos_scale = config["dof_pos_scale"]
    dof_vel_scale = config["dof_vel_scale"]
    action_scale = config["action_scale"]
    cmd_scale = np.array(config["cmd_scale"], dtype=np.float32)
    num_actions = config["num_actions"]
    num_obs = config["num_obs"]

    m = mujoco.MjModel.from_xml_path(xml_path)
    d = mujoco.MjData(m)
    m.opt.timestep = simulation_dt
    policy = torch.jit.load(policy_path)

    # Stato iniziale: identico al reset live (riga 0 del CSV = stato post-reset)
    mujoco.mj_resetData(m, d)
    qpos_keys = sorted((k for k in rows[0] if k.startswith('qpos_')), key=lambda x: int(x.split('_')[1]))
    qvel_keys = sorted((k for k in rows[0] if k.startswith('qvel_')), key=lambda x: int(x.split('_')[1]))
    d.qpos[:] = [float(rows[0][k]) for k in qpos_keys]
    d.qvel[:] = [float(rows[0][k]) for k in qvel_keys]

    cmd = np.zeros(3, dtype=np.float32)
    action = np.zeros(num_actions, dtype=np.float32)
    target_dof_pos = default_angles.copy()
    obs = np.zeros(num_obs, dtype=np.float32)

    total_steps = int(round(sim_times[-1] / simulation_dt))
    replayed = [d.qpos[0:3].copy()]  # riga 0: stato iniziale, come nel CSV live

    counter = 0
    for _ in range(total_steps):
        tau = sim_utils.pd_control(target_dof_pos, d.qpos[7:], kps,
                                   np.zeros_like(kds), d.qvel[6:], kds)
        d.ctrl[:] = tau
        mujoco.mj_step(m, d)

        counter += 1
        if counter % control_decimation == 0:
            sim_time = counter * simulation_dt
            cmd[:] = cmds[sim_utils.find_log_index(sim_times, sim_time)]

            qj = (d.qpos[7:] - default_angles) * dof_pos_scale
            dqj = d.qvel[6:] * dof_vel_scale
            gravity_orientation = sim_utils.get_gravity_orientation(d.qpos[3:7])
            omega = d.qvel[3:6] * ang_vel_scale

            period = 0.8
            phase = sim_time % period / period
            n = num_actions
            obs[:3] = omega
            obs[3:6] = gravity_orientation
            obs[6:9] = cmd * cmd_scale
            obs[9:9 + n] = qj
            obs[9 + n:9 + 2 * n] = dqj
            obs[9 + 2 * n:9 + 3 * n] = action
            obs[9 + 3 * n:9 + 3 * n + 2] = np.array([np.sin(2 * np.pi * phase),
                                                      np.cos(2 * np.pi * phase)])
            with torch.no_grad():
                action[:] = policy(torch.from_numpy(obs).unsqueeze(0)).numpy().squeeze()
            target_dof_pos[:] = action * action_scale + default_angles

            replayed.append(d.qpos[0:3].copy())

    replayed = np.array(replayed)
    compare_len = min(len(live_pos), len(replayed))
    sq_err = np.sum((live_pos[:compare_len] - replayed[:compare_len]) ** 2, axis=1)
    mse = float(np.mean(sq_err))
    rmse = float(np.sqrt(mse))

    print("\n--- Replay Evaluation Results ---")
    print(f"Compared steps: {compare_len}")
    print(f"MSE:  {mse:.8f} m^2")
    print(f"RMSE: {rmse:.6f} m")
    print(f"Max deviation: {np.sqrt(np.max(sq_err)):.6f} m")

    out_csv = os.path.join(bag_dir, f"{bag_name}_replay.csv")
    with open(out_csv, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(["pos_x", "pos_y", "pos_z"])
        w.writerows(replayed[:compare_len].tolist())
    print(f"Saved replayed trajectory to {out_csv}")

    if mse < 1e-4:
        print("\n[SUCCESS] Replay matches live trajectory (MSE negligible).")
        return True
    print("\n[WARNING] Trajectory mismatch: check that recording started from reset state.")
    return False


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag_name')
    parser.add_argument('--bag-dir', default=DEFAULT_BAG_DIR)
    parser.add_argument('--rl-gym', default=DEFAULT_RL_GYM)
    ns = parser.parse_args(args)
    ok = run_replay(ns.bag_name, os.path.expanduser(ns.bag_dir), os.path.expanduser(ns.rl_gym))
    raise SystemExit(0 if ok else 1)


if __name__ == '__main__':
    main()
```

- [ ] **Step 2: Verifica end-to-end della metrica replay**

Registra una nuova sessione (~15 s con movimento) dalla dashboard, poi:

```bash
ros2 run progetto_robotica replay_eval <bag_name>
```
Expected: `[SUCCESS] Replay matches live trajectory (MSE negligible).` con MSE < 1e-4.
Nota: se la live gira col viewer attivo, il real-time throttling NON influisce: cmd e stato sono campionati in sim-time. Se MSE non è ~0, verificare che MuJoCo sia la stessa versione in live e replay.

- [ ] **Step 3: Commit**

```bash
git add progetto_robotica/replay_eval.py
git commit -m "feat: deterministic replay with sim-time resampling and MSE evaluation"
```

---

### Task 6: Misura latenza di controllo (TwistStamped + ping WebSocket)

**Files:**
- Modify: `progetto_robotica/web_teleop.py`
- Modify: `progetto_robotica/mujoco_sim.py`
- Modify: `web/static/js/dashboard.js`, `web/templates/index.html`

- [ ] **Step 1: web_teleop pubblica TwistStamped con stamp**

In `web_teleop.py` sostituisci l'import di Twist con `from geometry_msgs.msg import TwistStamped` e:

```python
        self.cmd_vel_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
```

```python
    def publish_cmd_vel(self, vx: float, vy: float, wz: float):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = float(vx)
        msg.twist.linear.y = float(vy)
        msg.twist.angular.z = float(wz)
        self.cmd_vel_pub.publish(msg)
```

- [ ] **Step 2: mujoco_sim misura e pubblica la latenza**

In `mujoco_sim.py`: import `TwistStamped` da geometry_msgs.msg e `Float64` da std_msgs.msg; sostituisci la subscription:

```python
        self.cmd_vel_sub = self.create_subscription(TwistStamped, '/cmd_vel', self.cmd_vel_callback, 10)
        self.latency_pub = self.create_publisher(Float64, '/metrics/cmd_latency_ms', 10)
```

```python
    def cmd_vel_callback(self, msg: TwistStamped):
        self.cmd[0] = msg.twist.linear.x
        self.cmd[1] = msg.twist.linear.y
        self.cmd[2] = msg.twist.angular.z
        self.last_cmd_rx = time.monotonic()
        latency_ns = (self.get_clock().now() - rclpy.time.Time.from_msg(msg.header.stamp)).nanoseconds
        self.latency_pub.publish(Float64(data=latency_ns / 1e6))
```

Inoltre: aggiungi `import rclpy.time` agli import in testa al file, e in `__init__` (dopo `self.bag_dir = ...`) aggiungi `self.last_cmd_rx = time.monotonic()` (usato anche dal watchdog del Task 7).

- [ ] **Step 3: web_teleop inoltra la latenza alla dashboard**

Subscription in `__init__`:

```python
        self.create_subscription(Float64, '/metrics/cmd_latency_ms', self.latency_callback, 10)
```

```python
    def latency_callback(self, msg):
        with state_lock:
            telemetry_state['latency_ms'] = round(msg.data, 1)
```

Aggiungi `'latency_ms': 0.0,` allo stato iniziale `telemetry_state` e `Float64` all'import std_msgs. Handler ping WebSocket (dopo `handle_teleop_cmd`):

```python
@socketio.on('latency_ping')
def handle_latency_ping(data):
    emit('latency_pong', data)
```

- [ ] **Step 4: dashboard — card latenza + ping**

In `index.html`, dentro `state-metrics` dopo la metric "Posizione Z":

```html
                            <div class="metric">
                                <span class="metric-label">Latenza cmd</span>
                                <span class="metric-value" id="val-latency">—</span>
                            </div>
```

In `dashboard.js`:

```javascript
// Latency probe: WS round-trip / 2 + latenza ROS (web node -> simulatore)
let wsLatencyMs = 0;
setInterval(() => socket.emit('latency_ping', { t: performance.now() }), 1000);
socket.on('latency_pong', (data) => { wsLatencyMs = (performance.now() - data.t) / 2; });
```

e nel listener `socket.on('telemetry', ...)` dopo l'aggiornamento di `valPosz`:

```javascript
    const totalLatency = wsLatencyMs + (state.latency_ms || 0);
    document.getElementById('val-latency').innerText = `${totalLatency.toFixed(1)} ms`;
```

- [ ] **Step 5: Verifica manuale**

Rebuild + launch, muovi il robot da tastiera: la card "Latenza cmd" deve mostrare valori stabili < 50 ms. `ros2 topic echo /metrics/cmd_latency_ms` deve pubblicare durante la teleop.

- [ ] **Step 6: Commit**

```bash
git add progetto_robotica/web_teleop.py progetto_robotica/mujoco_sim.py web/static/js/dashboard.js web/templates/index.html
git commit -m "feat: command latency metric via TwistStamped and WebSocket ping"
```

---

### Task 7: Watchdog su /cmd_vel

**Files:**
- Modify: `progetto_robotica/mujoco_sim.py`

- [ ] **Step 1: Parametro e check nel sim loop**

In `__init__`: `self.declare_parameter('cmd_timeout', 0.5)` e `self.cmd_timeout = self.get_parameter('cmd_timeout').value` (con `self.last_cmd_rx = time.monotonic()` già presente dal Task 6).

In `run_sim_loop`, subito dopo il blocco `if self.reset_requested:`:

```python
                if self.cmd.any() and (time.monotonic() - self.last_cmd_rx) > self.cmd_timeout:
                    self.cmd[:] = 0.0
                    self.get_logger().warn("cmd_vel timeout: command zeroed for safety",
                                           throttle_duration_sec=5.0)
```

- [ ] **Step 2: Verifica manuale**

Launch, tieni premuto W e chiudi la tab del browser: entro ~0.5 s il robot deve fermarsi e il log mostra il warn.

- [ ] **Step 3: Commit**

```bash
git add progetto_robotica/mujoco_sim.py
git commit -m "feat: cmd_vel watchdog zeroes command on teleop disconnect"
```

---

### Task 8: Fall detection nel backend

**Files:**
- Modify: `progetto_robotica/mujoco_sim.py`
- Modify: `progetto_robotica/web_teleop.py`
- Modify: `web/static/js/dashboard.js`

- [ ] **Step 1: mujoco_sim pubblica /fall_detected**

In `__init__`:

```python
        self.declare_parameter('fall_threshold_deg', 35.0)
        self.fall_threshold_deg = self.get_parameter('fall_threshold_deg').value
        self.fall_pub = self.create_publisher(Bool, '/fall_detected', 10)
```

In `publish_telemetries`, dopo la pubblicazione dei contatti:

```python
        fallen = sim_utils.is_fallen(self.d.qpos[3], self.d.qpos[4],
                                     self.d.qpos[5], self.d.qpos[6],
                                     self.fall_threshold_deg)
        self.fall_pub.publish(Bool(data=fallen))
```

- [ ] **Step 2: web_teleop inoltra lo stato**

Aggiungi `'fall_detected': False,` a `telemetry_state`; in `__init__`:

```python
        self.create_subscription(Bool, '/fall_detected', self.fall_callback, 10)
```

```python
    def fall_callback(self, msg: Bool):
        with state_lock:
            telemetry_state['fall_detected'] = msg.data
```

In `start_recording`, aggiungi `"/fall_detected"` e `"/metrics/cmd_latency_ms"` alla lista topic di `ros2 bag record`.

- [ ] **Step 3: dashboard usa il valore backend**

In `dashboard.js` sostituisci il blocco soglia 35° client-side:

```javascript
    // Fall alert: calcolato dal backend (soglia roll/pitch parametrica, loggato in rosbag)
    if (state.fall_detected) {
        fallAlarm.classList.add('triggered');
    } else {
        fallAlarm.classList.remove('triggered');
    }
```

- [ ] **Step 4: Verifica manuale**

Launch, fai cadere il robot (comando laterale massimo prolungato o spinta dal viewer): overlay rosso + `ros2 topic echo /fall_detected` passa a `data: true`.

- [ ] **Step 5: Commit**

```bash
git add progetto_robotica/mujoco_sim.py progetto_robotica/web_teleop.py web/static/js/dashboard.js
git commit -m "feat: backend fall detection on /fall_detected, recorded in rosbag"
```

---

### Task 9: Streaming telemetria ≥30 Hz + emit fuori dal lock

**Files:**
- Modify: `progetto_robotica/web_teleop.py`

- [ ] **Step 1: Riscrivi `stream_telemetry_loop`**

Aggiungi `import copy` in testa al file, poi:

```python
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
```

- [ ] **Step 2: Verifica manuale**

Dashboard aperta 60 s: il badge "Hz" deve restare ≥ 30 stabilmente.

- [ ] **Step 3: Commit**

```bash
git add progetto_robotica/web_teleop.py
git commit -m "fix: compensated 33Hz telemetry loop, emit outside state lock"
```

---

### Task 10: README e verifica finale

**Files:**
- Create: `README.md`

- [ ] **Step 1: Scrivi `README.md`**

Contenuto minimo richiesto dal deliverable (adattare i dettagli a fine implementazione):

```markdown
# Teleoperazione e Web Dashboard per Unitree G1 (Traccia 10)

Teleoperazione del robot umanoide Unitree G1 in MuJoCo con dashboard web real-time:
telemetria (IMU, giunti, odometria, contatti piedi), alert caduta/flight-phase,
record/replay deterministico con valutazione MSE.

## Requisiti
- Windows 11 + WSL2 Ubuntu 24.04 (o Ubuntu 24.04 nativo)
- ROS 2 Jazzy
- Python 3.12, venv con `--system-site-packages`
- [unitree_rl_gym](https://github.com/unitreerobotics/unitree_rl_gym) clonato (policy pre-addestrata G1)

## Setup
    sudo apt install ros-jazzy-desktop ros-jazzy-rosbag2-storage-mcap
    git clone https://github.com/unitreerobotics/unitree_rl_gym ~/unitree_rl_gym
    export UNITREE_RL_GYM_PATH=~/unitree_rl_gym   # opzionale, default ~/unitree_rl_gym
    python3 -m venv --system-site-packages ~/venv
    ~/venv/bin/pip install -r requirements.txt
    mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src && git clone <questo-repo>
    cd ~/ros2_ws && source /opt/ros/jazzy/setup.bash && colcon build --symlink-install

## Esecuzione
    source ~/ros2_ws/install/setup.bash
    ros2 launch progetto_robotica teleop_sim_launch.py            # con viewer MuJoCo
    ros2 launch progetto_robotica teleop_sim_launch.py headless:=True

Dashboard: http://localhost:5000 — comandi WASD/QE (o gamepad), Start/Stop Record, Reset Sim.

## Replay e metriche
    ros2 run progetto_robotica replay_eval <bag_name>     # MSE vs sessione live
I bag rosbag2 (mcap) e i CSV sono in `~/progetto_robotica_bags` (parametro `bag_dir`).

## Architettura
- `mujoco_sim`: bridge MuJoCo + policy RL (unitree_rl_gym), pubblica /imu /joint_states
  /odom /contacts/* /fall_detected, sottoscrive /cmd_vel (TwistStamped) e /sim_reset.
- `web_teleop`: Flask-SocketIO + teleop tastiera/gamepad, streaming telemetria ~33 Hz.

## Test
    ~/venv/bin/python -m pytest test/ -v
```

- [ ] **Step 2: Verifica finale completa (checklist metriche della proposta)**

1. `pytest test/ -v` → tutti verdi.
2. Launch + dashboard: badge ≥ 30 Hz per 60 s.
3. Latenza cmd < 50 ms in dashboard.
4. Caduta → overlay + `/fall_detected` true; entrambi i piedi staccati → badge FLIGHT PHASE.
5. Record 15 s → `replay_eval` → MSE < 1e-4.
6. `grep -rn "/mnt/c\|/home/ergys" --include="*.py" .` → zero risultati.

- [ ] **Step 3: Commit**

```bash
git add README.md
git commit -m "docs: README with reproducible setup, run and metrics instructions"
```
