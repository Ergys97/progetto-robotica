Progetti per il corso di Robotica
Le seguenti proposte sono pensate per gruppi che possono essere composti da 1, 2 o 3
persone. La dimensione del gruppo deve riflettersi nella complessità del progetto: un
gruppo più numeroso deve produrre più scenari, una valutazione più ampia o
un’integrazione software più completa.
Per i progetti pratici (vedi sotto), è necessario avere un risultato dimostrabile, anche se
in forma prototipale. Non è richiesto sviluppare un sistema industriale completo, ma è
richiesto che il problema sia formulato chiaramente, che il codice sia eseguibile e che la
valutazione sia ragionata.
Per i progetti più teorici, è necessario avere una comprensione profonda dell’argomento
assegnato. Spesso, gli articoli proposti sono forniti anche di un’implementazione che
permette di valutare l’approccio nella pratica. In tal caso è richiesto dimostrare di avere
una consapevolezza critica dello strumento utilizzato
Regole principali e modalità di consegna
L’avvio del progetto deve essere concordato preventivamente con i docenti. Prima di
iniziare il lavoro, il gruppo deve inviare via email al sottoscritto e al dott. Luigi Gargioni
un documento sintetico di 1-2 pagine che descriva la proposta progettuale.
La proposta deve essere basata su uno dei temi suggeriti, eventualmente adattato al
numero di partecipanti, e deve indicare in modo chiaro:
● obiettivo del progetto;
● componenti software o strumenti che si intende usare;
● scenario o dominio di valutazione;
● risultati attesi e metriche di valutazione;
● suddivisione indicativa del lavoro, nel caso di gruppi da 2 o 3 persone.
Le fasi previste sono:
● invio e approvazione della proposta di progetto;
● svolgimento del progetto;
● registrazione all’appello tramite il portale;
● consegna del progetto via email al sottoscritto e al dott. Luigi Gargioni almeno 5
giorni lavorativi prima della data dell’appello;
● presentazione e discussione orale del progetto (in data d’esame, non fuori
sessione), per un massimo di 20 minuti.
La presentazione del progetto e l’eventuale prova orale possono svolgersi anche in
appelli distinti. La verbalizzazione avverrà nell’appello in cui entrambe le verifiche
saranno completate.
Le proposte di progetto presentate per l'anno accademico 2025/2026 saranno valide
fino alla sessione d'esame di Pasqua 2027.
Pratica e teoria possono essere svolte in sessioni separate, ma sempre prima la pratica
e poi la teoria. Entrambe valgono il 50% del voto, quindi hanno peso uguale.
Deliverable comuni
● relazione descrittiva del progetto, di massimo 5 pagine;
o Specifiche: A4, margini 2 cm, font 11 pt, interlinea 1.08, singola colonna
● repository con codice sorgente, configurazioni e istruzioni di esecuzione (ove
applicabile);
● demo riproducibile, preferibilmente in simulazione (ove applicabile);
● valutazione sperimentale con metriche semplici ma motivate (ove applicabile);
● discussione di limiti, assunzioni e possibili estensioni.
Esempi di carico lavorativo
● 1 studente: prototipo essenziale, un dominio o scenario, valutazione comparativa
più limitata.
● 2 studenti: prototipo più robusto, più istanze sperimentali, confronto tra almeno
due configurazioni o metodi.
● 3 studenti: architettura più completa, integrazione più estesa, benchmark più ampio e
analisi comparativa più solida.
È compito del gruppo presentare un'ipotesi di suddivisione dei lavori.
Progetti (molti possono essere personalizzati)
1 - (Pratico) Valutazione Sperimentale Stack NAV2 per robotica mobile
Analizzare sperimentalmente un aspetto dello stack di navigazione NAV2 per robotica
mobile. Lo studio parte dalla costruzione di un robot simulato diverso da quello usato
nei tutorial ufficiali, includendo modello URDF/SDF, trasformazioni TF, sensori,
odometria e configurazione base in Gazebo/RViz. La documentazione ufficiale di
riferimento è https://docs.nav2.org/index.html, in particolare le guide di setup del robot e
di integrazione con Gazebo.
Il focus dell’analisi sperimentale può riguardare, a scelta:
● Sensoristica e Odometria
● Mapping and Localisation
● Navigation and Control
Il progetto deve collegare le scelte implementative agli argomenti visti a lezione, ad
esempio locomozione, modelli di sensore, odometria, localizzazione probabilistica,
mappe a griglia, SLAM, path planning e controllo locale.
Attività principali
● modellare un robot mobile custom, con footprint, sensori e trasformazioni
coerenti;
● configurare una pipeline NAV2 funzionante in simulazione;
● scegliere un aspetto dello stack da analizzare in modo sperimentale;
● progettare alcuni scenari di test, ad esempio ambienti con corridoi, passaggi
stretti, ostacoli, mappe ambigue o waypoint multipli;
● confrontare almeno due configurazioni, algoritmi o condizioni sperimentali;
● valutare il sistema con metriche motivate, tra cui:
o tasso di successo della missione;
o tempo di navigazione;
o lunghezza del percorso;
o errore finale rispetto al goal;
o numero di replanning o recovery;
o errore di localizzazione o qualità della mappa, ove applicabile;
o comportamento in presenza di ostacoli o passaggi difficili.
2 - (Pratico) Task multi-agent con pianificazione simbolica, Gazebo e stack
di navigazione
Sviluppare un sistema multi-robot in cui un livello simbolico assegna task di
monitoraggio a più robot, mentre l’esecuzione dei movimenti è demandata a NAV2. Il
sistema deve considerare fallimenti locali (path non eseguibile) e prevedere una forma
di ripianificazione o riassegnazione. La pianificazione è fatta utilizzando un linguaggio
della famiglia PDDL. L’integrazione può avvenire attraverso lo Unified Planning
(https://unified-planning.readthedocs.io/en/latest/). Il task specifico deve essere
concordato. Esempi validi (ognuno rappresenterebbe un progetto indipendente):
● Braccio robotico che effettua pick and place di un oggetto in una posizione
specifica del tavolo. L’oggetto in questione era originariamente posseduto da un
altro robot che deve quindi trasportare l’oggetto in una posizione del tavolo in cui
il braccio robotico può prendere l’oggetto
● Esplorazione singolo agente per aumentare conoscenza della mappa
● Pulizia ambiente distribuiti. Abbiamo un numero di robot pulitori ed il task è di
condividere la pulizia dell’ambiente
● Blocks World in Gazebo
● Task a scelta
Attività principali:
● modellare robot, task, costi, tempi e capacità;
● monitorare completamento, fallimento o timeout;
● valutare il sistema su scenari di complessità crescente.
o numero di task completati (ove applicabile);
o tempo totale di missione;
o numero di riassegnazioni (ove applicabile);
o bilanciamento del carico tra robot (ove applicabile);
o robustezza rispetto ai fallimenti.
3 - (Pratico) Micromouse con simulatore di maze
Progetto adatto a chi è interessato alla pianificazione e al controllo di un robot mobile in
un ambiente discreto e parzialmente osservabile. L’obiettivo è sviluppare un algoritmo
per un robot Micromouse in grado di esplorare un labirinto, costruire una
rappresentazione interna delle pareti osservate e raggiungere la zona obiettivo
minimizzando il costo del percorso. Il progetto deve essere sviluppato utilizzando il
simulatore mms (https://github.com/mackorone/mms), che permette di testare algoritmi
di risoluzione del labirinto senza disporre di un robot fisico. Il linguaggio di
programmazione può essere scelto dal gruppo, purché sia compatibile con l’interfaccia
del simulatore.
Attività principali:
● studio del problema Micromouse e dell’interfaccia fornita da mms;
● implementazione di un algoritmo di esplorazione del maze, ad esempio wall
following, flood fill, A* incrementale o una strategia equivalente;
● costruzione e aggiornamento di una mappa interna del labirinto a partire dalle
osservazioni disponibili;
● gestione dei casi di dead-end, ritorno a celle già visitate e scelta del prossimo
obiettivo di esplorazione;
● valutazione su maze diversi, includendo almeno un confronto tra due strategie o
due configurazioni dell’algoritmo;
● analisi critica rispetto a
o numero di celle visitate;
o lunghezza del percorso finale;
o numero di mosse o turni effettuati;
o tempo di completamento della simulazione;
o robustezza rispetto a maze di complessità crescente.
4 - (Pratico) Heuristic Search per la pianificazione numerica
Progetto adatto a chi è interessato alla ricerca su tecniche di pianificazione avanzata. Il
focus è sullo sviluppo e sulla sperimentazione di strategie innovative di heuristic search
applicate alla pianificazione numerica. Il tema può essere personalizzato in base agli
interessi dello studente.
Attività principali
● Definizione condivisa del focus progettuale
● Sviluppo di nuove strategie euristiche
● Validazione su benchmark numerici e confronto con metodi esistenti
5 - (Pratico) Euristiche domain-dependent per pianificazione ibrida
temporale
Progetto adatto a chi è interessato alla ricerca su tecniche di pianificazione avanzata.
Sviluppare un sistema software per integrare euristiche e tecniche di guidance
domain-dependent in un planner ibrido temporale utilizzando la libreria Jpddlplus
(https://github.com/hstairs/jpddlplus). L’idea è di permettere all’utente l’agile
implementazione di informazioni specifiche per un dominio, e il confronto con una
baseline domain independent. Tra le tecniche che possono essere studiate, focalizzarsi
su: 1. euristiche -> Assegno un numero positivo ad uno stato 2. helpful actions ->
Individuo quali azioni sono utili per uno stato
Attivita’ principali:
● studio delle helpful actions
● studio della libreria, ed individuazione del sistema per la generazione ed uso di
meccanismi di guidance (euristica, helpful actions)
● definire una semplice interfaccia per euristiche custom;
● definire una semplice interfaccia per helpful actions;
● focus su due domini da concordare;
● confrontare le euristiche su istanze di complessità crescente, e su almeno due
domini diversi con analisi critica rispetto a
o tempo di pianificazione;
o numero di stati espansi;
o qualita’ del piano o makespan;
6 - (Teorico-Pratico) Domain Independent Dynamic Programming
Domain-independent dynamic programming (DIDP) è un nuovo approccio alla
modellazione di problemi combinatoriali compresi quelli di planning. Uno degli aspetti
chiave di DIDP è la potenzialità di poter rappresentare informazioni implicate in maniera
esplicita attraverso vincoli specifici. L’approccio viene presentato con un formalismo
simile a PDDL, e con delle tecniche di risoluzione che estendono algoritmi di ricerca
classica. Sebbene l’approccio non sia (ancora) usato in un contesto robotico.
Attività principali
● Studiare formalismo
● Studiare algoritmi risolutivi ed in particolare cost-algebraic A*
● Studiare il formalismo DyPDL (mezzo per specificare DIDP Problems)
● Modellare e risolvere 3 problemi a scelta dagli autori e riportare un'analisi
sperimentale
7 - (Teorico) Simple Temporal Logic per specifiche temporali in robotica
Studio della Signal Temporal Logic (STL) come formalismo per specificare e monitorare
vincoli temporali su missioni robotiche. STL è un formalismo nato per esplicitare vincoli
temporali su tracce. Tali tracce possono rappresentare stati o azioni dell’agente. In STL
le tracce sono temporizzate e i vincoli possono specificare la distanza temporale
metrica tra eventi particolari in un piano. Il formalismo rappresenta un ottimo candidato
per tutti quei task complessi fatti da uno o più robot. L’articolo in cui è stato definito il
formalismo è: Maler, Oded, and Dejan Nickovic. “Monitoring temporal properties of
continuous signals.” Formal Techniques, Modelling and Analysis of Timed and
Fault-Tolerant Systems. Springer, Berlin, Heidelberg, 2004. 152-166
Attività principali:
● studiare sintassi e semantica essenziale di STL;
● definire un piccolo insieme di formule;
● generare dei domini di esempio sul framework e fare un’analisi critica
8 - (Pratico) Task & Motion Planning in Robotica Centrata sull’Utente
Il progetto si basa su un sistema esistente
(https://github.com/luigigargioni/DTblocklyGPT), che combina LLMs e un’interfaccia
visuale basata su Blockly per permettere agli utenti non esperti di programmazione di
definire task per un robot. Verrà fornito in via confidenziale un articolo che spiega in
dettaglio il sistema.
Attraverso l’interazione con la chat del sistema, l’utente definisce un compito di
pick-and-place specificando i vari step (es. pick oggetto X, place in posizione Y, ripeti 3
volte); tale specifica viene tradotta in una rappresentazione in formato JSON, che
descrive il task in modo formale e comprensibile dal resto del sistema. Questo JSON
viene successivamente: (i) convertito in una rappresentazione visuale in Blockly (altra
struttura JSON proprietaria di Blockly) per poter visualizzare e modificare il task da
parte dell’utente; (ii) fatto il parsing per l’esecuzione del task sul digital twin del robot e/o
sul robot reale.
L’obiettivo del progetto è integrare questa architettura LLM-Blockly-DigitalTwin con la
pianificazione simbolica in PDDL e pianificatori automatici. In particolare, il lavoro
consisterà nel “connettere” due aspetti: da un lato, l’interazione uomo-robot mediata da
LLM e interfacce visuali orientate all’End-User Development; dall’altro, la pianificazione
simbolica formale, che permette la specifica di problemi di task planning in maniera
dichiarativa.
Attività principali:
● Definizione del dominio PDDL per scenari di manipolazione robotica, con
particolare riferimento a compiti di pick-and-place. Nell’implementazione attuale
c’è anche uno step di processing (es. shaking) tra il pick ed il place.
● Generazione automatica di problemi PDDL con mediazione LLM: a partire da un
intento dell’utente espresso in linguaggio naturale ed ad alto livello, approccio
goal-oriented (es. rimuovi i 5 oggetti dalla scrivania), e dalla conoscenza del
dominio PDDL, l’LLM dovrà produrre un’istanza di problema PDDL formalmente
valida e poi il pianificatore PDDL produrrà un piano coerente con la richiesta
dell’utente.
● Conversione del piano PDDL nel formato JSON dell’architettura, in modo da
rendere il piano eseguibile sul digital twin e/o sul robot reale tramite la pipeline
attuale.
● Supporto al flusso inverso (user-in-the-loop): l’utente potrà modificare il task
attraverso l’interfaccia Blockly (che opera su una rappresentazione proprietaria di
Blockly in JSON). Le modifiche verranno quindi riconvertite nel JSON del sistema
(funzionalità già presente) e poi in PDDL per verificare formalmente la validità del
task risultante rispetto al dominio definito (planning as verification).
Il risultato atteso è un framework integrato che consenta di:
● passare da obiettivi ad alto livello espressi dall’utente a piani simbolici
formalmente validi ed eseguibili dal robot o dal suo Digital Twin;
● mantenere una rappresentazione intermedia strutturata e “leggibile” come ponte
tra pianificazione simbolica e interazione uomo-robot.
Il progetto si concluderà con una valutazione sperimentale dell’approccio proposto,
discutendo:
● vantaggi in termini di usabilità, trasparenza ed explainability per utenti non
esperti di programmazione;
● un’analisi critica sulla combinazione LLM-PDDL;
● le prospettive di estensione verso scenari di Task and Motion Planning più
complessi.
9 - (Pratico) Architettura Ibrida per la Robotica Sociale
Il progetto ha l’obiettivo di analizzare e sperimentare con un’architettura ibrida fornita
(https://github.com/luigigargioni/llm4hrc) per la robotica sociale, in cui componenti
simboliche e neurali collaborano per gestire task complessi, interpretare l’ambiente e
interagire con l’utente in modo adattivo. L’attenzione è posta sull’uso combinato di
modelli linguistici e visivi avanzati per migliorare la pianificazione, il monitoraggio e la
reattività del robot. Verrà fornito in via confidenziale un articolo che spiega in dettaglio il
sistema.
Attività principali:
● Supporto al ragionamento contestuale tramite LLM avanzati: integrare modelli
linguistici in modalità multi-step (“reasoning mode”) per aggiornare lo stato del
task e proporre i prossimi passi in base al contesto operativo e all’interazione con
l’utente.
● Interpretazione visiva con modelli VLM (Vision-Language Models) e VLA
(Visual-Language-Action Model): valutare l’efficacia di modelli neurali in grado di
comprendere immagini e linguaggio per riconoscere oggetti, interpretare la scena
e guidare le azioni del robot.
● Verifica del progresso del task: definire criteri per il controllo dell’avanzamento
del task (es. condizioni raggiunte, step mancanti) e analizzarne affidabilità ed
errori (inclusi falsi positivi/negativi).
● Espansione dell’Event Planner: migliorare il sistema di pianificazione degli eventi
per gestire situazioni dinamiche, eccezioni o interventi esterni (es. azioni inattese
dell’utente) (vedasi approfondimento seguente).
● Controlli deterministici per il riconoscimento della situazione: integrare controlli
basati su regole per valutare la coerenza della situazione osservata e prendere
decisioni affidabili in ambienti incerti.
Approfondimento: Dal Reactive Scheduling al Goal-Driven Planning
Nel contesto della Human-Robot Interaction (HRI), i robot assistivi devono adattarsi
dinamicamente al comportamento umano, monitorare l’evoluzione della situazione e
prendere decisioni coerenti con obiettivi condivisi. L’architettura proposta include un
modulo Event Planner & Next Steps implementato come sistema esperto rule-based
con forward chaining, con alcune limitazioni tipiche dei sistemi reattivi:
● le attività vengono principalmente fornite al sistema sotto forma di task strutturati;
● non viene generato automaticamente un piano a partire da un goal astratto;
● non esiste una rappresentazione esplicita di precondizioni ed effetti delle azioni;
● la gestione temporale è basata su controllo reattivo piuttosto che su reasoning
temporale esplicito;
● il monitoraggio dell’esecuzione e la pianificazione risultano strettamente
accoppiati.
Attività principali:
1. Analisi dell’architettura attuale: lo studente dovrà analizzare il funzionamento
dell’attuale architettura, con particolare attenzione al modulo Event Planner, includendo:
rappresentazione delle attività, gestione di priorità e vincoli temporali, meccanismi di
attivazione delle regole, integrazione con il modulo di task progress, e modalità di
esecuzione delle attività pianificate.
2. Esplorazione di un approccio goal-driven: investigare come rappresentare goal, stato
del mondo e azioni; come generare sequenze di azioni da un obiettivo; come integrare
un meccanismo deliberativo nell’architettura esistente. Potranno essere esplorati
concetti come STRIPS-like operators, backward chaining, goal decomposition,
reasoning temporale ed execution monitoring.
3. Confronto critico tra approcci: confrontare sistemi rule-based reattivi, scheduler
deterministici, approcci deliberativi goal-driven e possibili integrazioni ibride, discutendo
vantaggi e svantaggi in termini di semplicità implementativa, trasparenza, flessibilità,
scalabilità e integrazione con LLM.
Risultati attesi
● un’analisi tecnica dell’architettura attuale;
● una valutazione critica dei limiti del modulo Event Planner;
● una piccola estensione sperimentale oppure un prototipo concettuale
goal-driven;
● una discussione comparativa tra approcci reattivi e deliberativi;
● eventuali test dimostrativi su semplici scenari assistivi.
Progetti su Robot Umanoide (Unitree G1)
Descrizione generale (versione EDU con 29 DoF) e SDK:
https://support.unitree.com/home/en/G1_developer/about_G1
Esempi e Progetti: https://github.com/unitreerobotics
Simulatore MuJoCo: https://github.com/unitreerobotics/unitree_mujoco
10 - (Pratico) Teleoperazione e dashboard di monitoraggio
Interfaccia per pilotare il G1 in MuJoCo da tastiera o gamepad, con una dashboard in
tempo reale che mostra lo stato del robot (IMU, joint states, velocità, contatti ai piedi).
Attività principali:
● Nodo ROS 2 in Python per leggere input da tastiera e pubblicare velocity
commands.
● Dashboard con rqt e/o web-based (Flask + plotly) con grafici live di sensori.
● Alert visivi: caduta rilevata (angolo IMU > soglia), contatto perso.
● Registrazione e replay di una sessione di teleoperazione.
11 - (Pratico) Obstacle avoidance reattivo senza mappa
Il G1 naviga verso un goal con coordinate note usando solo sensori di prossimità (raggi
laser o depth camera simulata), senza costruire nessuna mappa, logica puramente
reattiva.
Attività principali:
● Ambiente MuJoCo semplice con ostacoli statici di forme diverse.
● Implementazione e confronto di 2–3 algoritmi reattivi.
● Nessuno SLAM: goal dato come coordinata assoluta, sensori solo locali.
● Analisi su scenari “corridoio stretto” e “ostacolo a U” (casi limite noti).
12 - (Pratico) Task planning simbolico per compiti multi-step
Il G1 riceve un goal ad alto livello (es. “porta il pacco nella stanza B”) e lo decompone
autonomamente in azioni primitive, usando un planner simbolico. Il progetto si focalizza
su scenari semplici e un numero limitato di azioni.
Attività principali:
● Definizione di un dominio PDDL con 3-4 azioni (es. navigate-to, pick, place).
● Utilizzo di un planner simbolico esistente in Python (es. unified-planning).
● Esecuzione del piano prodotto sul robot simulato tramite ROS 2.
● Test con almeno uno scenario di fallimento e re-planning.
13 - (Pratico) Social navigation
Il G1 traccia la posizione di un umano simulato e deve seguirlo mantenendo distanza
sociale, anticipare i cambi di direzione e fermarsi se l’umano si ferma.
Attività principali:
● Umano simulato in MuJoCo con traiettorie scriptate o generate proceduralmente.
● Stima posizione umana da depth camera simulata.
● Legge di controllo per following.
● Metriche HRI: distanza media mantenuta, reattività a stop/inversioni.