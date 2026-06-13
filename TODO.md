# Future Developments & Test Scenarios

This document outlines the planned test scenarios and future extensions for the Unitree G1 teleoperation and telemetry project.

## 1. Scenario 1: Linear Corridor on Flat Terrain (Corridoio Lineare)
*Already implemented as the default simulation environment, but open to future enhancements:*
- [ ] **Automated Velocity Calibration**: Implement a ROS 2 node to automatically tune the command scale parameters so that the physical robot speed matches the commanded `/cmd_vel` inputs exactly.
- [ ] **Drift Analysis**: Measure and plot the lateral drift (Y-axis) during long forward walks to evaluate the stability of the reinforcement learning policy.
- [ ] **Odometry vs Ground Truth comparison**: Create a telemetry plot comparing wheel-like or visual odometry estimates against the exact MuJoCo ground truth coordinates (`d.qpos[0:3]`).

## 2. Scenario 2: Obstacle Course (Percorso ad Ostacoli Elementari)
*To be implemented in future test campaigns to validate safety alerts:*
- [ ] **Obstacle Integration in XML**: Create a custom world configuration in `scene.xml` containing:
  - Small steps/ledges (e.g., 2–4 cm high blocks).
  - Inclined planes/ramps (e.g., 10–15 degree slopes).
- [ ] **Foot Contact Responsiveness Test**: Verify that the dashboard's foot contact indicator switches color dynamically when stepping onto uneven terrains or when a foot is completely off the ground.
- [ ] **Flight Phase Alert Tuning**: Validate the detection threshold for "flight phase" (both feet off the ground simultaneously) and ensure the alert triggers correctly without false positives during standard walking.
- [ ] **Fall Detection Calibration**: Perform stress testing by walking the robot over higher obstacles until it falls, confirming that the tilt angle threshold (35°) triggers the visual alert overlay reliably.
