# Relazione finale - Traccia 10

## 1. Obiettivo

Sintesi del prototipo: teleoperazione Unitree G1 in MuJoCo, dashboard web real-time,
alert di sicurezza e record/replay deterministico.

## 2. Architettura

Descrivere i nodi ROS 2 principali:

- `mujoco_sim`: simulazione, policy RL, pubblicazione telemetria e metriche.
- `web_teleop`: dashboard Flask-SocketIO, input tastiera/gamepad e gestione record.
- `replay_eval`: replay deterministico e calcolo MSE traiettoria.

Inserire diagramma di flusso dei topic:

- `/cmd_vel`
- `/imu`
- `/joint_states`
- `/odom`
- `/contacts/left`, `/contacts/right`
- `/fall_detected`
- `/metrics/cmd_latency_ms`
- `/recording_status`
- `/sim_reset`

## 3. Scenari

- `flat`: calibrazione, telemetria, latenza e replay MSE.
- `obstacle_course`: contatti piedi, flight phase e fall detection.

## 4. Risultati sperimentali

Compilare dopo la raccolta metriche descritta in `docs/metrics/protocollo-metriche.md`.

| Metrica | Target | Valore osservato | Esito |
| --- | --- | --- | --- |
| Frequenza telemetria | >= 30 Hz | Da compilare | Da compilare |
| Latenza comando | < 50 ms | Da compilare | Da compilare |
| MSE replay | < 1e-4 m^2 | Da compilare | Da compilare |
| Alert contatti/flight/fall | Coerenti con scenario | Da compilare | Da compilare |

## 5. Analisi critica

Discutere limiti osservati, trade-off del profilo `record_profile`, stabilita della
policy nello scenario ostacoli e riproducibilita del setup.
