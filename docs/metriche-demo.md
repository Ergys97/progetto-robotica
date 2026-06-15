# Protocollo metriche demo

## Setup

Avviare sempre da workspace ROS 2 compilato:

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch progetto_robotica teleop_sim_launch.py headless:=True scenario:=flat record_profile:=metrics telemetry_hz:=50 csv_hz:=50 viewer_fps:=15
```

## Scope delle misure

La fedelta di replay viene misurata sullo scenario `flat`, dove la traiettoria e
piu adatta a una verifica deterministica della MSE. Lo scenario `obstacle_course`
viene usato per validare la reattivita degli alert: contatti dei piedi, fase di
volo e rilevamento caduta.

## Tabella risultati

| Scenario | Prova | Metrica | Target | Valore osservato | Esito |
| --- | --- | --- | --- | --- | --- |
| flat | Dashboard aperta 60 s | Frequenza telemetria | >= 30 Hz | Da compilare | Da compilare |
| flat | WASD/QE per 30 s | Latenza comando | < 50 ms | Da compilare | Da compilare |
| flat | Record 15 s + replay | MSE traiettoria | < 1e-4 m^2 | Da compilare | Da compilare |
| obstacle_course | Cammino su rampa/step | Contatti piedi | Cambiano coerentemente | Da compilare | Da compilare |
| obstacle_course | Perdita simultanea contatti | Flight phase | Badge visibile | Da compilare | Da compilare |
| obstacle_course | Caduta o tilt forzato | Fall detection | Overlay e /fall_detected=true | Da compilare | Da compilare |

## Export dashboard

La dashboard permette di scaricare il riassunto della sessione osservata tramite
`Scarica JSON` o `Scarica CSV`. Usare il JSON/CSV per compilare la tabella sopra
con frequenza telemetria, latenza, assetto massimo, quota Z e conteggi degli
eventi di sicurezza.

## Comandi

```bash
ros2 launch progetto_robotica teleop_sim_launch.py headless:=True scenario:=flat record_profile:=metrics telemetry_hz:=50 csv_hz:=50 viewer_fps:=15
ros2 run progetto_robotica replay_eval <bag_name>
ros2 launch progetto_robotica teleop_sim_launch.py headless:=True scenario:=obstacle_course record_profile:=minimal telemetry_hz:=50 csv_hz:=50 viewer_fps:=15
ros2 topic echo /fall_detected
ros2 topic echo /metrics/cmd_latency_ms
```
