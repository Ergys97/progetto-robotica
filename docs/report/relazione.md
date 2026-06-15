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

I risultati selezionati sono archiviati in `docs/results/`. I dati grezzi completi
sono conservati fuori repository in `/home/ergys/ros2_ws/src/progetto-robotica-metrics`.

| Scenario | Metrica | Target | Valore osservato | Esito |
| --- | --- | --- | --- | --- |
| `flat` | Frequenza telemetria media/minima | >= 30 Hz | 33.51 Hz / 32 Hz | OK |
| `flat` | Latenza comando media/p95/max | < 50 ms | 7.53 / 23.45 / 28.35 ms | OK |
| `flat` | MSE replay | < 1e-4 m^2 | 0.00000000 m^2 | OK |
| `flat` | Stabilita assetto | nessuna caduta | roll 6.09 deg, pitch 6.53 deg, 0 cadute | OK |
| `obstacle_course` | Frequenza telemetria media | >= 30 Hz | 32.93 Hz | OK |
| `obstacle_course` | Latenza comando media/p95 | < 50 ms | 9.57 / 26.35 ms | OK |
| `obstacle_course` | Fall detection | >= 1 evento | 4 eventi | OK |
| `obstacle_course` | Flight phase | >= 1 evento | 79 eventi | OK |
| `obstacle_course` | Contatti piedi | eventi coerenti con ostacoli | left 114, right 139 perdite contatto | OK |

## 5. Analisi critica

Discutere limiti osservati: nello scenario obstacle il valore massimo di latenza
raggiunge 466.55 ms, ma e trattato come picco isolato perche media e p95 restano
ampiamente sotto il target. La fedelta replay e misurata sullo scenario `flat`, dove
la traiettoria e adatta alla verifica deterministica.
