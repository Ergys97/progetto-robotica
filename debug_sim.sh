#!/usr/bin/env bash
source /opt/ros/jazzy/setup.bash
export PYTHONPATH=/home/ergys/venv/lib/python3.12/site-packages:$PYTHONPATH
/home/ergys/venv/bin/python3 -c "from progetto_robotica.mujoco_sim import MuJoCoSimNode; import rclpy; rclpy.init(); n = MuJoCoSimNode(); n.headless = True; n.run_sim_loop()"
