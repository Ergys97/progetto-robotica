# Scenario ostacoli, UI minimale e metriche - Design

## Obiettivo

Portare il progetto alla versione finale consigliata per la consegna universitaria: un prototipo allineato alla proposta accettata, con scenario piano e scenario a ostacoli elementari, dashboard piu sobria e una traccia chiara per raccogliere le metriche richieste.

## Scope approvato

Implementare l'opzione A:

- Un solo percorso a ostacoli elementare, selezionabile da launch.
- Cleanup del simulatore, inclusa rimozione delle stampe debug residue.
- Redesign frontend minimale e coerente con un progetto universitario/scientifico.
- Documentazione per esecuzione demo e raccolta metriche: Hz telemetria, latenza comando, MSE replay, fall/flight/contact alert.

Restano fuori scope:

- Due o piu percorsi a ostacoli.
- Calibrazione automatica velocita.
- Dockerizzazione completa.
- Risultati sperimentali numerici definitivi, perche richiedono esecuzione in WSL/ROS 2 con MuJoCo disponibile.

## Architettura scenario ostacoli

Il launch file riceve un parametro `scenario` con valori `flat` e `obstacle_course`. Lo scenario `flat` continua a usare il `scene.xml` originale di `unitree_rl_gym`.

Per `obstacle_course`, il nodo `mujoco_sim` non modifica il repository Unitree. Usa un helper Python testabile, `progetto_robotica.scene_builder`, che legge il `scene.xml` di base, risolve eventuali include relativi in path assoluti, inserisce piccoli ostacoli nel `worldbody` e scrive un XML generato sotto `bag_dir/_generated_scenes/`. Il nodo carica poi quell'XML generato.

Gli ostacoli saranno deliberatamente elementari:

- una rampa bassa, inclinata circa 8 gradi;
- tre piccoli step progressivi da circa 2-4 cm.

Questa scelta copre la proposta senza trasformare il progetto in tuning della policy RL. Se la policy cade, il comportamento e comunque utile per validare fall detection; se supera gli ostacoli, e utile per validare contatti e flight phase.

## Frontend

La dashboard resta una singola pagina Flask/Socket.IO/Plotly, ma viene resa piu sobria:

- lingua italiana;
- palette neutra chiara o scura senza glow marcati;
- rimozione del linguaggio "premium" e degli effetti glassmorphism;
- metriche principali in alto: Hz, latenza, roll, pitch, quota Z, stato contatti;
- controlli e registrazione piu compatti;
- grafici leggibili, non decorativi.

Non si introduce un framework frontend: sarebbe sovradimensionato rispetto al deliverable.

## Documentazione e metriche

Aggiungere una pagina `docs/metrics/protocollo-metriche.md` con protocollo di prova e tabella risultati da compilare durante la demo:

- scenario piano: telemetria, latenza, replay MSE;
- scenario ostacoli: contatti, flight phase, fall detection;
- comandi ROS/launch da usare e criterio di accettazione.

Il README deve spiegare come selezionare lo scenario:

```bash
ros2 launch progetto_robotica teleop_sim_launch.py scenario:=flat
ros2 launch progetto_robotica teleop_sim_launch.py scenario:=obstacle_course
```

## Verifica

Verifiche automatiche locali:

- `python -m pytest test -v` o equivalente nel venv WSL;
- test unitari per `scene_builder`;
- grep per verificare assenza di debug print nel simulatore.

Verifiche manuali in WSL/ROS 2:

- `colcon build --symlink-install`;
- launch headless di entrambi gli scenari;
- dashboard a `http://localhost:5000`;
- record/replay con MSE;
- controllo che gli alert contatti/flight/fall reagiscano nello scenario ostacoli.

## Rischi

- In questa sessione `wsl` risulta senza distribuzioni installate, quindi non posso validare build ROS/MuJoCo end-to-end.
- La policy G1 pre-addestrata potrebbe non superare ostacoli troppo aggressivi. Per questo lo scenario resta elementare e finalizzato alla validazione degli alert, non alla locomozione robusta su terreni complessi.
- Il redesign non deve nascondere i dati: la priorita resta leggibilita scientifica, non estetica.
