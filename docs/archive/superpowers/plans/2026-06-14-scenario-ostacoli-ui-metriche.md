# Scenario Ostacoli, UI Minimale e Metriche Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Aggiungere uno scenario MuJoCo a ostacoli elementari selezionabile da launch, pulire la demo e rendere dashboard/documentazione piu coerenti con una consegna universitaria.

**Architecture:** Lo scenario piano resta invariato. Lo scenario ostacoli viene generato a runtime partendo dal `scene.xml` configurato, senza modificare `unitree_rl_gym`, tramite un helper Python puro testabile. Il frontend resta HTML/CSS/JS statico servito da Flask-SocketIO, con redesign minimale senza introdurre framework.

**Tech Stack:** ROS 2 Jazzy, MuJoCo XML, Python `xml.etree.ElementTree`, Flask-SocketIO, Plotly.js, pytest.

---

## File Structure

- Create `progetto_robotica/scene_builder.py`: helper puro per validare scenario, risolvere include relativi e generare XML ostacoli.
- Create `test/test_scene_builder.py`: test unitari senza ROS/MuJoCo.
- Modify `progetto_robotica/mujoco_sim.py`: dichiarazione parametro `scenario`, selezione XML generato, rimozione debug print.
- Modify `launch/teleop_sim_launch.py`: nuovo launch argument `scenario`.
- Modify `web/templates/index.html`: testi italiani, struttura piu sobria, rimozione commenti/copy premium.
- Modify `web/static/css/style.css`: palette neutra, no glassmorphism/glow, card compatte.
- Modify `web/static/js/dashboard.js`: label coerenti, layout Plotly leggibile su palette minimale.
- Modify `README.md`: istruzioni scenario piano/ostacoli.
- Create `docs/metrics/protocollo-metriche.md`: protocollo prova e tabella risultati.
- Modify `TODO.md`: segnare lo scenario ostacoli come implementato, lasciando solo sviluppi futuri reali.

---

### Task 1: Scene Builder TDD

**Files:**
- Create: `test/test_scene_builder.py`
- Create: `progetto_robotica/scene_builder.py`

- [ ] **Step 1: Write failing tests**

Create `test/test_scene_builder.py`:

```python
from pathlib import Path

import pytest

from progetto_robotica.scene_builder import build_obstacle_scene, resolve_scene_xml


def test_flat_scenario_returns_original_xml(tmp_path):
    source = tmp_path / "scene.xml"
    source.write_text("<mujoco><worldbody /></mujoco>", encoding="utf-8")

    result = resolve_scene_xml(str(source), "flat", str(tmp_path / "out"))

    assert result == str(source)


def test_unknown_scenario_is_rejected(tmp_path):
    source = tmp_path / "scene.xml"
    source.write_text("<mujoco><worldbody /></mujoco>", encoding="utf-8")

    with pytest.raises(ValueError, match="Unsupported scenario"):
        resolve_scene_xml(str(source), "stairs", str(tmp_path / "out"))


def test_obstacle_scene_injects_expected_geoms_and_rewrites_include(tmp_path):
    model = tmp_path / "g1.xml"
    model.write_text("<mujoco model='g1' />", encoding="utf-8")
    source = tmp_path / "scene.xml"
    source.write_text(
        "<mujoco><include file='g1.xml' /><worldbody><geom name='floor' type='plane' /></worldbody></mujoco>",
        encoding="utf-8",
    )

    generated = Path(build_obstacle_scene(str(source), str(tmp_path / "out")))
    text = generated.read_text(encoding="utf-8")

    assert generated.name == "obstacle_course_scene.xml"
    assert f'file="{model.as_posix()}"' in text
    assert 'name="obstacle_ramp"' in text
    assert 'name="obstacle_step_1"' in text
    assert 'name="obstacle_step_2"' in text
    assert 'name="obstacle_step_3"' in text
```

- [ ] **Step 2: Run tests and verify RED**

Run:

```bash
python -m pytest test/test_scene_builder.py -v
```

Expected: FAIL with `ModuleNotFoundError` or import error for `progetto_robotica.scene_builder`.

- [ ] **Step 3: Implement minimal helper**

Create `progetto_robotica/scene_builder.py`:

```python
"""Helpers for generating MuJoCo scenario XML files without modifying vendor assets."""

from __future__ import annotations

import os
import xml.etree.ElementTree as ET
from pathlib import Path


SUPPORTED_SCENARIOS = {"flat", "obstacle_course"}


def resolve_scene_xml(xml_path: str, scenario: str, output_dir: str) -> str:
    """Return the XML path that MuJoCo should load for the requested scenario."""
    if scenario not in SUPPORTED_SCENARIOS:
        supported = ", ".join(sorted(SUPPORTED_SCENARIOS))
        raise ValueError(f"Unsupported scenario '{scenario}'. Supported scenarios: {supported}")
    if scenario == "flat":
        return xml_path
    return build_obstacle_scene(xml_path, output_dir)


def build_obstacle_scene(base_xml_path: str, output_dir: str) -> str:
    """Generate an obstacle-course scene from a base MuJoCo XML scene."""
    base_path = Path(os.path.expanduser(base_xml_path)).resolve()
    tree = ET.parse(base_path)
    root = tree.getroot()

    _rewrite_includes_to_absolute_paths(root, base_path.parent)
    worldbody = root.find("worldbody")
    if worldbody is None:
        worldbody = ET.SubElement(root, "worldbody")

    _remove_existing_generated_obstacles(worldbody)
    _append_obstacles(worldbody)

    target_dir = Path(os.path.expanduser(output_dir)).resolve()
    target_dir.mkdir(parents=True, exist_ok=True)
    generated_path = target_dir / "obstacle_course_scene.xml"
    tree.write(generated_path, encoding="utf-8", xml_declaration=True)
    return str(generated_path)


def _rewrite_includes_to_absolute_paths(root: ET.Element, base_dir: Path) -> None:
    for include in root.findall(".//include"):
        include_file = include.get("file")
        if not include_file:
            continue
        include_path = Path(os.path.expanduser(include_file))
        if not include_path.is_absolute():
            include_path = (base_dir / include_path).resolve()
        include.set("file", include_path.as_posix())


def _remove_existing_generated_obstacles(worldbody: ET.Element) -> None:
    generated_names = {"obstacle_ramp", "obstacle_step_1", "obstacle_step_2", "obstacle_step_3"}
    for geom in list(worldbody.findall("geom")):
        if geom.get("name") in generated_names:
            worldbody.remove(geom)


def _append_obstacles(worldbody: ET.Element) -> None:
    ET.SubElement(
        worldbody,
        "geom",
        {
            "name": "obstacle_ramp",
            "type": "box",
            "size": "0.70 0.45 0.035",
            "pos": "2.0 0 0.035",
            "euler": "0 0.139626 0",
            "rgba": "0.45 0.48 0.52 1",
            "friction": "1.0 0.005 0.0001",
        },
    )
    for index, (x_pos, height) in enumerate(((3.0, 0.02), (3.35, 0.03), (3.70, 0.04)), start=1):
        ET.SubElement(
            worldbody,
            "geom",
            {
                "name": f"obstacle_step_{index}",
                "type": "box",
                "size": f"0.13 0.45 {height / 2:.3f}",
                "pos": f"{x_pos:.2f} 0 {height / 2:.3f}",
                "rgba": "0.36 0.39 0.43 1",
                "friction": "1.0 0.005 0.0001",
            },
        )
```

- [ ] **Step 4: Run tests and verify GREEN**

Run:

```bash
python -m pytest test/test_scene_builder.py -v
```

Expected: 3 passed.

- [ ] **Step 5: Commit**

```bash
git add progetto_robotica/scene_builder.py test/test_scene_builder.py
git commit -m "feat: generate obstacle-course MuJoCo scene"
```

---

### Task 2: Wire Scenario Into Launch and Simulator

**Files:**
- Modify: `progetto_robotica/mujoco_sim.py`
- Modify: `launch/teleop_sim_launch.py`

- [ ] **Step 1: Add simulator parameter and generated XML selection**

In `progetto_robotica/mujoco_sim.py`, change the import:

```python
from progetto_robotica import scene_builder, sim_utils
```

After `self.declare_parameter('fall_threshold_deg', 35.0)` add:

```python
        self.declare_parameter('scenario', 'flat')
```

After reading `config_path`, add:

```python
        scenario = self.get_parameter('scenario').value
```

Before `self.m = mujoco.MjModel.from_xml_path(xml_path)`, add:

```python
        generated_scene_dir = os.path.join(self.bag_dir, "_generated_scenes")
        xml_path = scene_builder.resolve_scene_xml(xml_path, scenario, generated_scene_dir)
        self.get_logger().info(f"Loading MuJoCo scenario '{scenario}' from: {xml_path}")
```

- [ ] **Step 2: Remove debug print block**

Delete this block from `evaluate_policy`:

```python
        if self.counter == 10:
            print("LIVE SIM OBS AT 10:", self.obs.tolist())
            print("LIVE SIM ACTION AT 10:", self.action.tolist())
            print("LIVE SIM TARGET AT 10:", self.target_dof_pos.tolist())
```

- [ ] **Step 3: Add launch argument**

In `launch/teleop_sim_launch.py`, add to `args`:

```python
        DeclareLaunchArgument('scenario', default_value='flat',
                              description='Scenario MuJoCo: flat oppure obstacle_course'),
```

In `sim_node.parameters`, add:

```python
            'scenario': LaunchConfiguration('scenario'),
```

- [ ] **Step 4: Run focused checks**

Run:

```bash
python -m pytest test/test_scene_builder.py test/test_sim_utils.py -v
rg -n "LIVE SIM OBS|LIVE SIM ACTION|LIVE SIM TARGET" progetto_robotica/mujoco_sim.py
```

Expected: tests pass; `rg` returns no matches.

- [ ] **Step 5: Commit**

```bash
git add progetto_robotica/mujoco_sim.py launch/teleop_sim_launch.py
git commit -m "feat: select flat or obstacle MuJoCo scenario from launch"
```

---

### Task 3: Scientific Frontend Redesign

**Files:**
- Modify: `web/templates/index.html`
- Modify: `web/static/css/style.css`
- Modify: `web/static/js/dashboard.js`

- [ ] **Step 1: Update HTML copy and remove premium/glass wording**

Use Italian labels:

- title: `Unitree G1 - Teleoperazione e metriche`
- header: `Dashboard sperimentale`
- subtitle: `Telemetria MuJoCo/ROS 2 in tempo reale`
- fall overlay: `ALLARME CADUTA`
- session title: `Registrazione sessione`

Replace each `class="card glassmorphism"` with `class="card"`.

- [ ] **Step 2: Replace CSS theme**

Update `:root` values in `web/static/css/style.css`:

```css
:root {
    --bg-base: #f5f7fa;
    --bg-panel: #ffffff;
    --bg-muted: #eef2f7;
    --border-color: #d8dee8;
    --color-primary: #2457a6;
    --color-danger: #b42318;
    --color-success: #14804a;
    --color-warning: #b7791f;
    --text-main: #182230;
    --text-muted: #667085;
    --font-sans: 'Inter', sans-serif;
    --font-mono: 'JetBrains Mono', monospace;
    --sidebar-width: 248px;
}
```

Then remove glow-heavy styling from `.glassmorphism`, `.brand-icon`, `.status-indicator.online`, `.status-indicator.offline`, `.pad-btn:hover`, `.record-banner`, and `.alarm-content`. Keep borders and background colors flat.

- [ ] **Step 3: Update Plotly colors**

In `web/static/js/dashboard.js`, set neutral layout colors:

```javascript
const chartColors = {
    paper: '#ffffff',
    plot: '#ffffff',
    grid: '#e4e7ec',
    axis: '#667085',
};
```

Use `chartColors` in `imuLayout` and `jointsLayout` for `paper_bgcolor`, `plot_bgcolor`, axis color and grid color.

- [ ] **Step 4: Static verification**

Run:

```bash
rg -n "Premium|glassmorphism|CRITICAL ALERT|Live Telemetry Dashboard|Dashboard Grid System" web
```

Expected: no matches for removed wording/classes except comments that still describe generic dashboard sections if intentionally retained.

- [ ] **Step 5: Commit**

```bash
git add web/templates/index.html web/static/css/style.css web/static/js/dashboard.js
git commit -m "style: make dashboard minimal and scientific"
```

---

### Task 4: Documentation and Metrics Protocol

**Files:**
- Modify: `README.md`
- Modify: `TODO.md`
- Create: `docs/metrics/protocollo-metriche.md`

- [ ] **Step 1: Update README launch commands**

Replace the execution block with:

```markdown
source ~/ros2_ws/install/setup.bash

# Scenario piano
ros2 launch progetto_robotica teleop_sim_launch.py scenario:=flat

# Scenario a ostacoli elementari
ros2 launch progetto_robotica teleop_sim_launch.py scenario:=obstacle_course

# Modalita headless
ros2 launch progetto_robotica teleop_sim_launch.py headless:=True scenario:=obstacle_course
```

- [ ] **Step 2: Add scenario section to README**

Add:

```markdown
## Scenari di prova
- `flat`: piano regolare, usato per calibrazione comandi, telemetria e replay MSE.
- `obstacle_course`: rampa bassa e piccoli step progressivi, usati per validare contatti piedi, flight phase e fall detection.
```

- [ ] **Step 3: Create metrics protocol**

Create `docs/metrics/protocollo-metriche.md`:

```markdown
# Protocollo metriche demo

## Setup

Avviare sempre da workspace ROS 2 compilato:

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch progetto_robotica teleop_sim_launch.py headless:=True scenario:=flat
```

## Tabella risultati

| Scenario | Prova | Metrica | Target | Valore osservato | Esito |
| --- | --- | --- | --- | --- | --- |
| flat | Dashboard aperta 60 s | Frequenza telemetria | >= 30 Hz | Da compilare | Da compilare |
| flat | WASD/QE per 30 s | Latenza comando | < 50 ms | Da compilare | Da compilare |
| flat | Record 15 s + replay | MSE traiettoria | < 1e-4 m^2 | Da compilare | Da compilare |
| obstacle_course | Cammino su rampa/step | Contatti piedi | Cambiano coerentemente | Da compilare | Da compilare |
| obstacle_course | Perdita simultanea contatti | Flight phase | Badge visibile | Da compilare | Da compilare |
| obstacle_course | Caduta o tilt forzato | Fall detection | Overlay e /fall_detected=true | Da compilare | Da compilare |

## Comandi

```bash
ros2 launch progetto_robotica teleop_sim_launch.py headless:=True scenario:=flat
ros2 run progetto_robotica replay_eval <bag_name>
ros2 launch progetto_robotica teleop_sim_launch.py headless:=True scenario:=obstacle_course
ros2 topic echo /fall_detected
ros2 topic echo /metrics/cmd_latency_ms
```
```

- [ ] **Step 4: Update TODO**

Move obstacle-course XML integration from unchecked future item to implemented status, leaving calibration and deeper analysis as future work.

- [ ] **Step 5: Commit**

```bash
git add README.md TODO.md docs/metrics/protocollo-metriche.md
git commit -m "docs: describe scenarios and demo metrics protocol"
```

---

### Task 5: Final Verification

**Files:**
- All changed files

- [ ] **Step 1: Run local unit tests**

Run:

```bash
python -m pytest test -v
```

Expected in a configured Python/WSL venv: all tests pass.

- [ ] **Step 2: Run static scans**

Run:

```bash
rg -n "LIVE SIM OBS|LIVE SIM ACTION|LIVE SIM TARGET|Premium|glassmorphism|CRITICAL ALERT" progetto_robotica web
rg -n "/mnt/c|/home/ergys" progetto_robotica launch web README.md TODO.md docs/metrics/protocollo-metriche.md
```

Expected: no matches for debug/UI-premium terms; no hardcoded personal paths in production/docs except intentionally documented default `~/...`.

- [ ] **Step 3: Run WSL/ROS checks when WSL is available**

Run in WSL with ROS 2 Jazzy:

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch progetto_robotica teleop_sim_launch.py headless:=True scenario:=flat
ros2 launch progetto_robotica teleop_sim_launch.py headless:=True scenario:=obstacle_course
```

Expected: both launches start without MuJoCo XML load errors.

- [ ] **Step 4: Commit final polish if needed**

```bash
git status --short
git diff --stat
```

If additional fixes were needed:

```bash
git add <fixed-files>
git commit -m "fix: final scenario and dashboard polish"
```

---

## Plan Self-Review

- Spec coverage: scenario ostacoli, cleanup debug, UI minimale, README/TODO and metriche protocol are covered by Tasks 1-4.
- Placeholder scan: no `TBD` or deferred implementation placeholders remain; WSL-only checks are explicit because WSL is unavailable in this session.
- Type consistency: `resolve_scene_xml(xml_path, scenario, output_dir)` is introduced in Task 1 and used with the same signature in Task 2.
