# Protocollo metriche demo

## Setup

Avviare sempre da workspace ROS 2 compilato:

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch progetto_robotica teleop_sim_launch.py headless:=True scenario:=flat
```

## Tabella risultati

| Scenario | Prova | Metrica | Target | Valore osservato | Esito |
| --- | --- | --- | --- | --- | --- |
| flat | Dashboard aperta 60 s | Frequenza telemetria | >= 30 Hz | Da compilare | Da compilare |
| flat | WASD/QE per 30 s | Latenza comando | < 50 ms | Da compilare | Da compilare |
| flat | Record 15 s + replay | MSE traiettoria | < 1e-4 m^2 | Da compilare | Da compilare |
| obstacle_course | Cammino su rampa/step | Contatti piedi | Cambiano coerentemente | Da compilare | Da compilare |
| obstacle_course | Perdita simultanea contatti | Flight phase | Badge visibile | Da compilare | Da compilare |
| obstacle_course | Caduta o tilt forzato | Fall detection | Overlay e /fall_detected=true | Da compilare | Da compilare |

## Comandi

```bash
ros2 launch progetto_robotica teleop_sim_launch.py headless:=True scenario:=flat
ros2 run progetto_robotica replay_eval <bag_name>
ros2 launch progetto_robotica teleop_sim_launch.py headless:=True scenario:=obstacle_course
ros2 topic echo /fall_detected
ros2 topic echo /metrics/cmd_latency_ms
```
