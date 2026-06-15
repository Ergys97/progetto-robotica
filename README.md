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
*(Nota: in questo ambiente e consigliato il build standard sopra descritto. Alcune versioni di `setuptools` non supportano correttamente l'installazione editable usata da `colcon build --symlink-install`.)*

## Esecuzione
Comandi consigliati da WSL, dal root del repository:
```bash
# Scenario piano
bash scripts/run_demo.sh flat

# Scenario a ostacoli elementari
bash scripts/run_demo.sh obstacle

# Modalita headless
bash scripts/run_demo.sh obstacle --headless

# Ricompila prima del launch
bash scripts/run_demo.sh flat --build
```

Lo script attiva `~/venv` se presente, carica ROS 2 Jazzy, carica
`~/ros2_ws/install/setup.bash` e avvia il launch corretto.

Comandi ROS equivalenti:
```bash
source ~/ros2_ws/install/setup.bash

# Scenario piano
ros2 launch progetto_robotica teleop_sim_launch.py scenario:=flat

# Scenario a ostacoli elementari
ros2 launch progetto_robotica teleop_sim_launch.py scenario:=obstacle_course

# Modalita headless
ros2 launch progetto_robotica teleop_sim_launch.py headless:=True scenario:=obstacle_course

# Registrazione stabile per replay/metriche
ros2 launch progetto_robotica teleop_sim_launch.py headless:=True scenario:=flat record_profile:=metrics telemetry_hz:=50.0 csv_hz:=50.0 viewer_fps:=15.0
```

Dashboard accessibile all'indirizzo: http://localhost:5000
- Comandi di movimento: tasti WASD/QE (o gamepad USB), pulsante Stop (Spazio)
- Gestione sessione: Start/Stop Record, Reset Sim.
- Export metriche: al termine della registrazione il riassunto viene salvato in
  `~/progetto_robotica_bags/metrics/`; i pulsanti Scarica JSON/CSV restano disponibili
  per il download manuale.

## Scenari di prova
- `flat`: piano regolare, usato per calibrazione comandi, telemetria e replay MSE.
- `obstacle_course`: rampa bassa e piccoli step progressivi, usati per validare contatti piedi, flight phase e fall detection.

## Replay e metriche
Per eseguire il replay deterministico e calcolare le metriche (MSE traiettoria):
```bash
ros2 run progetto_robotica replay_eval <bag_name>     # MSE vs sessione live
```
I file mcap di rosbag2 e i file CSV di telemetria vengono salvati in `~/progetto_robotica_bags` (personalizzabile tramite parametro `bag_dir`).
I riassunti JSON/CSV della dashboard vengono salvati in `~/progetto_robotica_bags/metrics`.
Il protocollo di raccolta risultati e disponibile in `docs/metrics/protocollo-metriche.md`.

Per ridurre il lag durante la registrazione:
- `record_profile:=minimal` registra comandi, contatti, caduta e latenza.
- `record_profile:=metrics` aggiunge `/odom` ed e il default consigliato per replay/metriche.
- `record_profile:=full` registra anche `/imu` e `/joint_states`, utile per debug ma piu pesante.
- `telemetry_hz:=50.0` limita i topic pubblicati dal simulatore.
- `csv_hz:=50.0` limita il logging CSV.
- `viewer_fps:=15.0` limita il sync del viewer; per misure stabili usare anche `headless:=True`.

## Architettura
- `mujoco_sim`: bridge MuJoCo + policy RL (unitree_rl_gym), pubblica `/imu`, `/joint_states`, `/odom`, `/contacts/*`, `/fall_detected` e `/metrics/cmd_latency_ms`. Sottoscrive `/cmd_vel` (`TwistStamped`) e `/sim_reset`.
- `web_teleop`: Flask-SocketIO + teleop tastiera/gamepad, streaming telemetria a ~33 Hz verso la dashboard e inoltro dei comandi.
- `sim_utils`: modulo Python contenente la logica matematica pura condivisa (controllo PD, calcolo gravità corpo, roll/pitch da quaternione, rilevamento caduta).

## Test
Esecuzione degli unit test (senza dipendenze ROS):
```bash
~/venv/bin/python -m pytest test/ -v
```

## Documentazione
- `docs/proposal/`: proposta di progetto e testo estratto della traccia.
- `docs/metrics/`: protocollo di misura e tabella risultati da compilare durante la demo.
- `docs/results/`: risultati sperimentali selezionati per la relazione (`flat` e `obstacle_course`).
- `docs/report/`: destinazione della relazione finale.
- `docs/troubleshooting/`: note operative non essenziali al runtime.
- `docs/archive/`: appunti di sviluppo e piani storici non necessari alla consegna.
