# Protocollo metriche demo

## Setup

Avviare sempre da workspace ROS 2 compilato:

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch progetto_robotica teleop_sim_launch.py headless:=True scenario:=flat record_profile:=metrics telemetry_hz:=50.0 csv_hz:=50.0 viewer_fps:=15.0
```

## Scope delle misure

La fedelta di replay viene misurata sullo scenario `flat`, dove la traiettoria e
piu adatta a una verifica deterministica della MSE. Lo scenario `obstacle_course`
viene usato per validare la reattivita degli alert: contatti dei piedi, fase di
volo e rilevamento caduta.

I test selezionati per la relazione sono archiviati in:

- `docs/results/flat/`
- `docs/results/obstacle/`

## Tabella risultati

| Scenario | Prova | Metrica | Target | Valore osservato | Esito |
| --- | --- | --- | --- | --- | --- |
| flat | Dashboard aperta 55.73 s | Frequenza telemetria | >= 30 Hz | media 33.51 Hz, minima 32 Hz | OK |
| flat | WASD/QE durante registrazione | Latenza comando | < 50 ms | media 7.53 ms, p95 23.45 ms, max 28.35 ms | OK |
| flat | Record 42.4 s + replay | MSE traiettoria | < 1e-4 m^2 | 0.00000000 m^2 | OK |
| obstacle_course | Cammino su rampa/step | Contatti piedi | Cambiano coerentemente | 114 perdite contatto sinistro, 139 destro | OK |
| obstacle_course | Perdita simultanea contatti | Flight phase | Badge visibile | 79 eventi | OK |
| obstacle_course | Caduta o tilt forzato | Fall detection | Overlay e /fall_detected=true | 4 eventi, roll max 126.63 deg, pitch max 89.28 deg | OK |

## Export dashboard

Alla pressione di `Stop Record`, la dashboard invia automaticamente il riassunto
della sessione al backend, che salva:

```bash
~/progetto_robotica_bags/metrics/<bag_name>_metrics.json
~/progetto_robotica_bags/metrics/<bag_name>_metrics.csv
```

I pulsanti `Scarica JSON` e `Scarica CSV` restano disponibili come fallback manuale.
Usare il JSON/CSV per compilare la tabella sopra con frequenza telemetria, latenza,
assetto massimo, quota Z e conteggi degli eventi di sicurezza.

## Comandi

```bash
ros2 launch progetto_robotica teleop_sim_launch.py headless:=True scenario:=flat record_profile:=metrics telemetry_hz:=50.0 csv_hz:=50.0 viewer_fps:=15.0
ros2 run progetto_robotica replay_eval <bag_name>
ros2 launch progetto_robotica teleop_sim_launch.py headless:=True scenario:=obstacle_course record_profile:=minimal telemetry_hz:=50.0 csv_hz:=50.0 viewer_fps:=15.0
ros2 topic echo /fall_detected
ros2 topic echo /metrics/cmd_latency_ms
```
