# Presentazione e demo - Teleoperazione ROS 2 di Unitree G1

**Durata totale:** 20 minuti  
**Struttura consigliata:** 15 minuti presentazione + 5 minuti demo finale  
**Titolo:** Teleoperazione ROS 2 di Unitree G1 con dashboard realtime e validazione sperimentale

## Obiettivo della presentazione

L'obiettivo è mostrare che il progetto non è solo una simulazione avviata in MuJoCo,
ma una piccola architettura robotica completa:

- controllo del robot tramite teleoperazione;
- simulazione Unitree G1 in MuJoCo;
- integrazione con ROS 2;
- dashboard web realtime;
- registrazione, replay e metriche sperimentali;
- validazione su due scenari distinti.

Messaggio principale da far passare:

> Il sistema permette di controllare Unitree G1 in simulazione, osservare in tempo
> reale lo stato del robot e validare quantitativamente telemetria, latenza, replay
> e alert di sicurezza.

## Scaletta da 15 minuti

### 1. Introduzione e obiettivo - 2 minuti

Punti da dire:

- Il progetto segue la Traccia 10: teleoperazione e dashboard web per Unitree G1.
- L'ambiente scelto è MuJoCo, con ROS 2 Jazzy come middleware.
- Il focus non è solo il movimento del robot, ma anche osservabilità, logging e
  validazione sperimentale.

Frase utile:

> Ho impostato il progetto come un sistema robotico osservabile: il robot riceve
> comandi, pubblica telemetria ROS 2, la dashboard mostra lo stato in tempo reale e
> le sessioni possono essere registrate e riprodotte.

### 2. Architettura software - 4 minuti

Mostrare la Figura 1 della relazione.

Componenti da spiegare:

- `mujoco_sim`
  - carica il modello Unitree G1;
  - integra la policy di locomozione;
  - riceve `/cmd_vel`;
  - pubblica `/imu`, `/joint_states`, `/odom`, `/contacts/*`, `/fall_detected`,
    `/metrics/cmd_latency_ms`.
- `web_teleop`
  - legge input tastiera/gamepad;
  - pubblica comandi di velocità;
  - serve la dashboard Flask-SocketIO;
  - invia dati realtime al browser.
- `replay_eval`
  - legge una sessione registrata;
  - riproduce la traiettoria;
  - calcola MSE/RMSE.
- `rosbag2`
  - registra topic selezionati;
  - permette replay e analisi offline.

Frase utile:

> Ho separato il sistema in nodi con responsabilità distinte: simulazione, interfaccia
> web e valutazione sperimentale. Questo rende più semplice controllare, registrare
> e validare il comportamento del robot.

### 3. Dashboard e teleoperazione - 2 minuti

Punti da dire:

- La dashboard è accessibile da browser su `http://localhost:5000`.
- Visualizza telemetria, assetto, contatti dei piedi, stato caduta e latenza.
- I comandi arrivano da tastiera o gamepad.
- Il sistema include reset simulazione e gestione record.

Frase utile:

> La dashboard serve come strumento di controllo e osservabilità: durante la demo
> posso vedere se il robot è stabile, se perde contatto con il terreno e se la
> latenza dei comandi resta entro il target.

### 4. Scenari sperimentali - 2 minuti

Scenari:

- `flat`
  - piano regolare;
  - usato per telemetria, latenza e replay;
  - scenario più adatto alla misura deterministica.
- `obstacle_course`
  - rampa e piccoli step;
  - usato per contatti piedi, flight phase e fall detection;
  - scenario più adatto a stressare gli alert.

Frase utile:

> Ho separato gli scenari per evitare di usare un singolo test per obiettivi troppo
> diversi: il piano regolare misura la stabilità del sistema, mentre gli ostacoli
> servono a verificare gli alert.

### 5. Metriche e risultati - 4 minuti

Mostrare tabella risultati e figure della relazione.

Metriche principali:

| Metrica | Target | Risultato selezionato |
| --- | --- | --- |
| Frequenza telemetria flat | >= 30 Hz | 33.51 Hz media, 32 Hz minima |
| Latenza comando flat | < 50 ms | media 7.53 ms, p95 23.45 ms |
| Replay flat | MSE < 1e-4 m^2 | 0.00000000 m^2 |
| Frequenza telemetria obstacle | >= 30 Hz | 32.93 Hz media |
| Latenza obstacle | < 50 ms su media/p95 | media 9.57 ms, p95 26.35 ms |
| Fall detection obstacle | >= 1 evento | 4 eventi |
| Flight phase obstacle | >= 1 evento | 79 eventi |
| Perdite contatto obstacle | eventi coerenti | left 114, right 139 |

Punto da chiarire sulla latenza:

> Nello scenario obstacle compare un picco massimo di 466.55 ms, ma media e p95
> restano sotto target. Lo considero un outlier legato a transitori/registrazione,
> non il comportamento tipico del sistema.

Punto da chiarire sul replay:

> Ho validato il replay sullo scenario flat perché è il dominio più controllato:
> permette di confrontare traiettoria live e traiettoria riprodotta senza introdurre
> cadute, reset e discontinuità tipiche dello scenario obstacle.

Perché l'MSE è esattamente 0 (domanda probabile):

> L'MSE risulta esattamente zero perché la sessione registra, oltre alla traiettoria,
> lo stato iniziale (qpos/qvel post-reset) e i target di controllo PD applicati a ogni
> passo (colonne `target_` nel CSV). In replay questi target vengono rialimentati nella
> stessa fisica MuJoCo, che è deterministica: a parità di stato iniziale e di sequenza
> di controlli la traiettoria coincide bit a bit. Quindi lo zero non è un caso fortunato,
> ma la conferma che la pipeline di registrazione e riproduzione è fedele e che la
> simulazione è riproducibile. Esiste anche un percorso che ricalcola la policy dalle
> osservazioni (MSE non nullo, più sensibile al rumore numerico): l'ho lasciato come
> estensione, ma per la metrica di fedeltà richiesta dalla traccia il replay dei target
> è il confronto corretto.

Punto da chiarire sul grafico assetto/quota:

> Il grafico obstacle è a segmenti perché il log contiene reset e discontinuità.
> I segmenti sono mostrati in sequenza relativa per evitare di sovrapporre campioni
> con lo stesso `sim_time`.

### 6. Limiti e sviluppi futuri - 1 minuto

Limiti:

- validazione solo in simulazione;
- nessun test su hardware reale;
- velocità comandata non calibrata automaticamente rispetto alla velocità effettiva;
- odometria non confrontata sistematicamente con ground truth MuJoCo.

Sviluppi futuri:

- calibrazione automatica dei comandi;
- confronto odometria vs ground truth;
- analisi drift laterale;
- adattamento a robot reale o simulazione distribuita.

Frase conclusiva:

> Il prototipo raggiunge gli obiettivi della traccia: teleoperazione, dashboard,
> logging, replay e validazione quantitativa sono integrati in un flusso unico e
> riproducibile.

## Demo finale da 5 minuti

### Obiettivo demo

Mostrare rapidamente:

1. avvio della simulazione;
2. dashboard web;
3. teleoperazione;
4. alert/telemetria;
5. posizione dei risultati e della relazione.

### Preparazione prima dell'esame

Aprire già due terminali:

- terminale 1: root del repository;
- terminale 2: eventuale comandi di controllo ROS o replay.

Verificare prima:

```bash
python3 -m pytest test/ -v
MPLCONFIGDIR=/tmp/matplotlib python3 scripts/generate_report_figures.py
MPLCONFIGDIR=/tmp/matplotlib python3 scripts/build_report_pdf.py
```

### Demo consigliata

#### Minuto 0:00-1:00 - Avvio scenario flat

Comando:

```bash
bash scripts/run_demo.sh flat
```

Dire:

> Avvio lo scenario piano, usato per telemetria, latenza e replay. Lo script carica
> l'ambiente ROS 2, il workspace installato e il launch del progetto.

Aprire dashboard:

```text
http://localhost:5000
```

#### Minuto 1:00-2:30 - Teleoperazione e dashboard

Azioni:

- muovere il robot con WASD/QE;
- mostrare grafici e indicatori dashboard;
- mostrare contatti, assetto e latenza.

Dire:

> La dashboard riceve dati realtime dal nodo ROS 2 e permette di osservare lo stato
> del robot mentre viene teleoperato.

#### Minuto 2:30-3:30 - Record e metriche

Azioni:

- mostrare pulsante Start/Stop Record se pratico;
- indicare dove finiscono i risultati.

Dire:

> Le sessioni possono essere registrate con rosbag2 e la dashboard salva anche un
> riepilogo JSON/CSV delle metriche.

Risultati selezionati nel repository:

```text
docs/results/flat/
docs/results/obstacle/
```

#### Minuto 3:30-4:30 - Scenario obstacle oppure risultati già pronti

Se il tempo e l'ambiente grafico sono stabili:

```bash
bash scripts/run_demo.sh obstacle
```

Altrimenti mostrare direttamente risultati/grafici:

```text
docs/report/relazione_finale.pdf
docs/report/figures/eventi_obstacle.png
docs/report/figures/assetto_quota_obstacle.png
```

Dire:

> Lo scenario obstacle serve soprattutto a validare gli alert: contatti dei piedi,
> flight phase e caduta.

#### Minuto 4:30-5:00 - Chiusura

Dire:

> La demo mostra il flusso completo: comando del robot, telemetria realtime,
> registrazione e risultati quantitativi. La relazione riporta i valori selezionati
> e i limiti del sistema.

## Fallback se la demo live ha problemi

Se MuJoCo o la grafica WSL danno problemi, usare questa sequenza:

1. mostrare `README.md` per comandi di esecuzione;
2. mostrare `docs/report/relazione_finale.pdf`;
3. mostrare i grafici in `docs/report/figures/`;
4. mostrare i risultati JSON/CSV in `docs/results/`;
5. spiegare che i test automatici passano con `40 passed`.

Frase pronta:

> In caso di problemi grafici dell'ambiente, il progetto resta riproducibile dai
> comandi documentati e i risultati sperimentali selezionati sono già salvati nel
> repository.

## File e cartelle da consegnare al professore

### Da includere nella cartella progetto

Includere questi file e cartelle:

```text
README.md
requirements.txt
package.xml
setup.py
setup.cfg
resource/
launch/
progetto_robotica/
web/
scripts/
test/
docs/proposal/
docs/metrics/
docs/results/
docs/report/
docs/presentation/
docs/troubleshooting/
TODO.md
```

Motivazione:

- `README.md`: istruzioni setup, esecuzione, replay e test.
- `package.xml`, `setup.py`, `setup.cfg`, `resource/`: metadati pacchetto ROS 2.
- `launch/`: file di lancio ROS 2.
- `progetto_robotica/`: codice sorgente principale.
- `web/`: dashboard HTML/CSS/JS.
- `scripts/`: script demo e generazione relazione/grafici.
- `test/`: test automatici.
- `docs/proposal/`: traccia e proposta.
- `docs/metrics/`: protocollo misure.
- `docs/results/`: risultati selezionati leggeri JSON/CSV.
- `docs/report/`: relazione finale PDF, Markdown e figure.
- `docs/presentation/`: scaletta presentazione e demo.
- `docs/troubleshooting/`: note operative.
- `TODO.md`: sviluppi futuri dichiarati.

### Da non includere, salvo richiesta esplicita

Non includere nella consegna principale:

```text
.git/
.pytest_cache/
__pycache__/
build/
install/
log/
~/progetto_robotica_bags/
```

Motivazione:

- `.git/`: non necessario se consegni una cartella compressa.
- `.pytest_cache/`, `__pycache__/`: cache locali.
- `build/`, `install/`, `log/`: output generati da colcon.
- `~/progetto_robotica_bags/`: rosbag e log grezzi possono essere pesanti.

### Dati grezzi opzionali

Se il professore chiede anche i rosbag grezzi, creare un archivio separato con:

```text
~/progetto_robotica_bags/bag_20260615_160043/
~/progetto_robotica_bags/bag_20260615_160043.csv
~/progetto_robotica_bags/bag_20260615_160043_replay.csv
~/progetto_robotica_bags/bag_20260615_164534/
~/progetto_robotica_bags/bag_20260615_164534.csv
~/progetto_robotica_bags/metrics/bag_20260615_160043_metrics.json
~/progetto_robotica_bags/metrics/bag_20260615_160043_metrics.csv
~/progetto_robotica_bags/metrics/bag_20260615_164534_metrics.json
~/progetto_robotica_bags/metrics/bag_20260615_164534_metrics.csv
```

Per la consegna standard, però, bastano i risultati leggeri già inclusi in
`docs/results/`.

## Checklist finale prima di inviare

Eseguire:

```bash
python3 -m pytest test/ -v
MPLCONFIGDIR=/tmp/matplotlib python3 scripts/generate_report_figures.py
MPLCONFIGDIR=/tmp/matplotlib python3 scripts/build_report_pdf.py
```

Controllare che esistano:

```text
docs/report/relazione_finale.pdf
docs/report/figures/architettura_ros2.png
docs/report/figures/latenza_comandi.png
docs/report/figures/eventi_obstacle.png
docs/report/figures/assetto_quota_obstacle.png
docs/results/flat/summary.md
docs/results/obstacle/summary.md
```

Comando consigliato per creare archivio consegna dalla directory superiore:

```bash
cd /home/ergys/ros2_ws/src
tar --exclude='progetto-robotica/.git' \
    --exclude='progetto-robotica/.pytest_cache' \
    --exclude='progetto-robotica/**/__pycache__' \
    -czf progetto-robotica-consegna.tar.gz progetto-robotica
```
