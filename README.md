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
```bash
sudo apt install ros-jazzy-desktop ros-jazzy-rosbag2-storage-mcap
git clone https://github.com/unitreerobotics/unitree_rl_gym ~/unitree_rl_gym
export UNITREE_RL_GYM_PATH=~/unitree_rl_gym   # opzionale, default ~/unitree_rl_gym

# 1. Crea ed installa le dipendenze nell'ambiente virtuale
python3 -m venv --system-site-packages ~/venv
~/venv/bin/pip install -r requirements.txt

# 2. Configura il workspace ROS 2
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone <questo-repo>

# 3. Compila il workspace usando l'interprete dell'ambiente virtuale per generare le shebang corrette
source ~/venv/bin/activate
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws
python3 -m colcon build
```
*(Nota: Se preferisci usare `colcon build --symlink-install` per non dover ricompilare ad ogni modifica dei file Python, assicurati che la versione di `setuptools` nel venv supporti le installazioni editable, altrimenti esegui il build standard sopra descritto. Gli eseguibili saranno pronti sotto `install/progetto_robotica/bin/`).*

## Esecuzione
```bash
source ~/ros2_ws/install/setup.bash

# Scenario piano
ros2 launch progetto_robotica teleop_sim_launch.py scenario:=flat

# Scenario a ostacoli elementari
ros2 launch progetto_robotica teleop_sim_launch.py scenario:=obstacle_course

# Modalita headless
ros2 launch progetto_robotica teleop_sim_launch.py headless:=True scenario:=obstacle_course
```

Dashboard accessibile all'indirizzo: http://localhost:5000
- Comandi di movimento: tasti WASD/QE (o gamepad USB), pulsante Stop (Spazio)
- Gestione sessione: Start/Stop Record, Reset Sim.

## Scenari di prova
- `flat`: piano regolare, usato per calibrazione comandi, telemetria e replay MSE.
- `obstacle_course`: rampa bassa e piccoli step progressivi, usati per validare contatti piedi, flight phase e fall detection.

## Replay e metriche
Per eseguire il replay deterministico e calcolare le metriche (MSE traiettoria):
```bash
ros2 run progetto_robotica replay_eval <bag_name>     # MSE vs sessione live
```
I file mcap di rosbag2 e i file CSV di telemetria vengono salvati in `~/progetto_robotica_bags` (personalizzabile tramite parametro `bag_dir`).

## Architettura
- `mujoco_sim`: bridge MuJoCo + policy RL (unitree_rl_gym), pubblica `/imu`, `/joint_states`, `/odom`, `/contacts/*`, `/fall_detected` e `/metrics/cmd_latency_ms`. Sottoscrive `/cmd_vel` (`TwistStamped`) e `/sim_reset`.
- `web_teleop`: Flask-SocketIO + teleop tastiera/gamepad, streaming telemetria a ~33 Hz verso la dashboard e inoltro dei comandi.
- `sim_utils`: modulo Python contenente la logica matematica pura condivisa (controllo PD, calcolo gravità corpo, roll/pitch da quaternione, rilevamento caduta).

## Test
Esecuzione degli unit test (senza dipendenze ROS):
```bash
~/venv/bin/python -m pytest test/ -v
```
