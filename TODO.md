# Future Developments & Test Scenarios

This document outlines the planned test scenarios and future extensions for the Unitree G1 teleoperation and telemetry project.

## 1. Scenario 1: Linear Corridor on Flat Terrain (Corridoio Lineare)
*Implemented as the default simulation environment:*
- [x] **Flat Terrain Walking**: Standard flat regular surface for baseline walk, telemetry streaming, and trajectory MSE replay.

## 2. Scenario 2: Obstacle Course (Percorso ad Ostacoli Elementari)
*Implemented in the `codex/scenario-ostacoli-ui-metriche` branch to validate safety alerts:*
- [x] **Obstacle Integration in XML**: Dynamically generated using the `scene_builder` Python helper, injecting a 3.5cm ramp and three progressive steps (2cm, 3cm, 4cm) at runtime.
- [x] **Foot Contact Responsiveness Test**: Foot contact indicators switch color dynamically according to uneven terrain.
- [x] **Flight Phase Alert Tuning**: Handled via simultaneous loss of foot contacts, showing a dynamic badge in the dashboard.
- [x] **Fall Detection Calibration**: Validated tilt threshold (35°) showing the overlay alert and publishing `/fall_detected`.

## 3. Future Developments
- [ ] **Automated Velocity Calibration**: Implement a ROS 2 node to tune command scale parameters automatically so that physical robot speed matches commanded `/cmd_vel` inputs exactly.
- [ ] **Drift & Kinematic Analysis**: Measure and plot lateral drift (Y-axis) and joint effort during long forward walks.
- [ ] **Odometry vs Ground Truth Comparison**: Create a telemetry plot comparing estimated odometry against exact MuJoCo ground truth coordinates (`d.qpos[0:3]`).
