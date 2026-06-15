# Risultati sperimentali selezionati

Questa cartella contiene solo gli artefatti leggeri versionati per la relazione:
summary, JSON e CSV aggregati della dashboard.

I dati grezzi voluminosi sono conservati fuori repository in:

```text
/home/ergys/ros2_ws/src/progetto-robotica-metrics
```

## Test selezionati

| Scenario | Bag | Cartella risultati |
| --- | --- | --- |
| `flat` | `bag_20260615_160043` | `docs/results/flat/` |
| `obstacle_course` | `bag_20260615_164534` | `docs/results/obstacle/` |

## Test obstacle confrontato

Il candidato `bag_20260615_165515` e stato confrontato ma non selezionato per la
relazione: era piu lungo, ma presentava meno eventi fall/flight e un picco di latenza
massimo piu alto.
