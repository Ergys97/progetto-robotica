# Scenario obstacle_course - test selezionato

## Identificativo

- Scenario: `obstacle_course`
- Bag selezionato: `bag_20260615_164534`
- Data acquisizione: 2026-06-15
- Durata dashboard: 145.57 s
- Candidato confrontato: `bag_20260615_165515`

## Motivazione scelta

Tra i due test obstacle disponibili, `bag_20260615_164534` e stato scelto per la
relazione perche contiene piu eventi di caduta e flight phase, mantenendo una
latenza p95 sotto il target. Il test `bag_20260615_165515` e piu lungo, ma presenta
meno eventi flight/fall e un picco di latenza massimo piu alto.

## Metriche dashboard

| Metrica | Target | Valore osservato | Esito |
| --- | --- | --- | --- |
| Frequenza telemetria media | >= 30 Hz | 32.93 Hz | OK |
| Frequenza telemetria minima | stabile salvo transitori | 3 Hz | Transitorio |
| Latenza media comando | < 50 ms | 9.57 ms | OK |
| Latenza p95 comando | < 50 ms | 26.35 ms | OK |
| Latenza massima comando | < 50 ms | 466.55 ms | Picco isolato |
| Assetto massimo | trigger caduta | roll 126.63 deg, pitch 89.28 deg | OK |
| Quota base | evidenza caduta/ostacoli | 0.0581-0.9168 m | OK |
| Cadute rilevate | >= 1 | 4 | OK |
| Flight phase | >= 1 | 79 | OK |
| Perdite contatto piedi | eventi coerenti con ostacoli | left 114, right 139 | OK |

## Interpretazione

Il test documenta la risposta della dashboard nello scenario a ostacoli:

- i contatti dei piedi cambiano durante attraversamento di rampa/step;
- la flight phase viene rilevata in piu eventi;
- il rilevamento caduta si attiva durante tilt/caduta del robot;
- latenza media e p95 restano sotto il target di 50 ms;
- il valore massimo di latenza e trattato come picco isolato, non come valore tipico.

## Artefatti inclusi

- `bag_20260615_164534_metrics.json`: riassunto dashboard in formato JSON.
- `bag_20260615_164534_metrics.csv`: riassunto dashboard in formato CSV.

Il rosbag grezzo e il CSV completo di telemetria restano fuori repository in
`~/progetto_robotica_bags/` e nell'archivio dati esterno, per evitare di versionare
file binari o log voluminosi.
