# Scenario flat - test selezionato

## Identificativo

- Scenario: `flat`
- Bag: `bag_20260615_160043`
- Data acquisizione: 2026-06-15
- Durata dashboard: 55.73 s
- Durata replay: 42.4 s di tempo simulato

## Metriche dashboard

| Metrica | Target | Valore osservato | Esito |
| --- | --- | --- | --- |
| Frequenza telemetria media | >= 30 Hz | 33.51 Hz | OK |
| Frequenza telemetria minima | >= 30 Hz | 32 Hz | OK |
| Latenza media comando | < 50 ms | 7.53 ms | OK |
| Latenza p95 comando | < 50 ms | 23.45 ms | OK |
| Latenza massima comando | < 50 ms | 28.35 ms | OK |
| Assetto massimo | stabilita su piano | roll 6.09 deg, pitch 6.53 deg | OK |
| Quota base | stabile su piano | 0.7532-0.7906 m | OK |
| Cadute rilevate | 0 | 0 | OK |
| Flight phase | 0 attesi su piano | 0 | OK |

## Replay deterministico

Comando:

```bash
ros2 run progetto_robotica replay_eval bag_20260615_160043
```

Risultato:

```text
Loaded 2119 telemetry frames (42.4s of sim time).

--- Replay Evaluation Results ---
Compared steps: 2119
MSE:  0.00000000 m^2
RMSE: 0.000000 m
Max deviation: 0.000000 m

[SUCCESS] Replay matches live trajectory (MSE negligible).
```

## Artefatti inclusi

- `bag_20260615_160043_metrics.json`: riassunto dashboard in formato JSON.
- `bag_20260615_160043_metrics.csv`: riassunto dashboard in formato CSV.

Il rosbag grezzo e il CSV completo di telemetria restano fuori repository in
`~/progetto_robotica_bags/`, per evitare di versionare file binari o log voluminosi.
