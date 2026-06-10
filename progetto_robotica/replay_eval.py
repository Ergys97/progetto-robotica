#!/usr/bin/env python3
import os
import sys
import csv
import numpy as np
import torch
import mujoco
import yaml

def run_replay(bag_name):
    bag_dir = "/mnt/c/Users/ergys/Desktop/Git_Repositories/progetto-robotica/bags"
    csv_path = os.path.join(bag_dir, f"{bag_name}.csv")
    
    if not os.path.exists(csv_path):
        print(f"Error: Log file not found at {csv_path}")
        return False
        
    print(f"Loading live log file: {csv_path}")
    
    # Read CSV data
    log_data = []
    field_names = []
    with open(csv_path, 'r') as f:
        reader = csv.DictReader(f)
        field_names = reader.fieldnames
        for row in reader:
            frame = {
                'timestamp': float(row['timestamp']),
                'pos_x': float(row['pos_x']),
                'pos_y': float(row['pos_y']),
                'pos_z': float(row['pos_z']),
                'cmd_vx': float(row['cmd_vx']),
                'cmd_vy': float(row['cmd_vy']),
                'cmd_wz': float(row['cmd_wz'])
            }
            for k, v in row.items():
                if k.startswith('qpos_') or k.startswith('qvel_'):
                    frame[k] = float(v)
            log_data.append(frame)
            
    if not log_data:
        print("Error: Log file is empty.")
        return False
        
    print(f"Successfully loaded {len(log_data)} telemetry frames.")
    
    # Load MuJoCo and configuration parameters
    xml_path = '/home/ergys/unitree_rl_gym/resources/robots/g1_description/scene.xml'
    policy_path = '/home/ergys/unitree_rl_gym/deploy/pre_train/g1/motion.pt'
    config_path = '/home/ergys/unitree_rl_gym/deploy/deploy_mujoco/configs/g1.yaml'
    
    with open(config_path, 'r') as f:
        config = yaml.load(f, Loader=yaml.FullLoader)
        
    simulation_dt = config["simulation_dt"]
    control_decimation = config["control_decimation"]
    kps = np.array(config["kps"], dtype=np.float32)
    kds = np.array(config["kds"], dtype=np.float32)
    default_angles = np.array(config["default_angles"], dtype=np.float32)
    
    ang_vel_scale = config["ang_vel_scale"]
    dof_pos_scale = config["dof_pos_scale"]
    dof_vel_scale = config["dof_vel_scale"]
    action_scale = config["action_scale"]
    cmd_scale = np.array(config["cmd_scale"], dtype=np.float32)
    
    num_actions = config["num_actions"]
    num_obs = config["num_obs"]
    
    # Initialize MuJoCo and Policy
    m = mujoco.MjModel.from_xml_path(xml_path)
    d = mujoco.MjData(m)
    m.opt.timestep = simulation_dt
    
    policy = torch.jit.load(policy_path)
    
    # Reset simulation to defaults
    mujoco.mj_resetData(m, d)
    
    # Set initial position and posture from the first logged frame
    qpos_keys = sorted([k for k in field_names if k.startswith('qpos_')], key=lambda x: int(x.split('_')[1]))
    qvel_keys = sorted([k for k in field_names if k.startswith('qvel_')], key=lambda x: int(x.split('_')[1]))
    
    if qpos_keys and qvel_keys:
        init_qpos = [log_data[0][k] for k in qpos_keys]
        init_qvel = [log_data[0][k] for k in qvel_keys]
        d.qpos[:] = init_qpos
        d.qvel[:] = init_qvel
        print("Initialized replay state with logged qpos and qvel.")
    else:
        init_x = log_data[0]['pos_x']
        init_y = log_data[0]['pos_y']
        init_z = log_data[0]['pos_z']
        d.qpos[0:3] = [init_x, init_y, init_z]
        print("Initialized replay position only (fallback).")
    
    # Teleoperation velocity cmd setup
    cmd = np.zeros(3, dtype=np.float32)
    
    # Replay variables
    action = np.zeros(num_actions, dtype=np.float32)
    target_dof_pos = default_angles.copy()
    obs = np.zeros(num_obs, dtype=np.float32)
    
    replayed_positions = []
    
    # Start simulation loop (running headlessly)
    total_steps = int((log_data[-1]['timestamp'] - log_data[0]['timestamp']) / simulation_dt)
    print(f"Replaying simulation for {total_steps} steps (~{total_steps * simulation_dt:.1f}s)...")
    
    start_time_log = log_data[0]['timestamp']
    
    for step in range(total_steps):
        sim_time = step * simulation_dt
        
        # Sample command from log closest to current simulation time
        log_index = min(int(sim_time / 0.02), len(log_data) - 1)
        current_frame = log_data[log_index]
        
        cmd[0] = current_frame['cmd_vx']
        cmd[1] = current_frame['cmd_vy']
        cmd[2] = current_frame['cmd_wz']
        
        # Apply PD Control
        tau = (target_dof_pos - d.qpos[7:]) * kps + (np.zeros_like(kds) - d.qvel[6:]) * kds
        d.ctrl[:] = tau
        
        # Step physics
        mujoco.mj_step(m, d)
        
        # Policy decimation loop (50Hz)
        if step % control_decimation == 0:
            qj = d.qpos[7:]
            dqj = d.qvel[6:]
            quat = d.qpos[3:7]
            omega = d.qvel[3:6]
            
            # Formulate observations
            qj = (qj - default_angles) * dof_pos_scale
            dqj = dqj * dof_vel_scale
            
            # Gravity vector mapping
            qw, qx, qy, qz = quat[0], quat[1], quat[2], quat[3]
            gravity_orientation = np.array([
                2 * (-qz * qx + qw * qy),
                -2 * (qz * qy + qw * qx),
                1 - 2 * (qw * qw + qz * qz)
            ])
            omega = omega * ang_vel_scale
            
            period = 0.8
            count = step * simulation_dt
            phase = count % period / period
            sin_phase = np.sin(2 * np.pi * phase)
            cos_phase = np.cos(2 * np.pi * phase)
            
            obs[:3] = omega
            obs[3:6] = gravity_orientation
            obs[6:9] = cmd * cmd_scale
            obs[9 : 9 + num_actions] = qj
            obs[9 + num_actions : 9 + 2 * num_actions] = dqj
            obs[9 + 2 * num_actions : 9 + 3 * num_actions] = action
            obs[9 + 3 * num_actions : 9 + 3 * num_actions + 2] = np.array([sin_phase, cos_phase])
            
            obs_tensor = torch.from_numpy(obs).unsqueeze(0)
            with torch.no_grad():
                action = policy(obs_tensor).numpy().squeeze()
                
            target_dof_pos = action * action_scale + default_angles
            
            # Store position corresponding to log data step rate
            replayed_positions.append({
                'pos_x': d.qpos[0],
                'pos_y': d.qpos[1],
                'pos_z': d.qpos[2]
            })
            
    # Calculate Mean Squared Error (MSE)
    print("\n--- Replay Evaluation Results ---")
    
    # Align lengths
    compare_len = min(len(log_data), len(replayed_positions))
    
    sq_errors = []
    for i in range(compare_len):
        live_pos = np.array([log_data[i]['pos_x'], log_data[i]['pos_y'], log_data[i]['pos_z']])
        rep_pos = np.array([replayed_positions[i]['pos_x'], replayed_positions[i]['pos_y'], replayed_positions[i]['pos_z']])
        sq_err = np.sum((live_pos - rep_pos) ** 2)
        sq_errors.append(sq_err)
        
    mse = np.mean(sq_errors)
    max_err = np.sqrt(np.max(sq_errors))
    
    print(f"Total compared steps: {compare_len}")
    print(f"Mean Squared Error (MSE): {mse:.8f} m^2")
    print(f"Root Mean Squared Error (RMSE): {np.sqrt(mse):.6f} m")
    print(f"Maximum Deviation: {max_err:.6f} m")
    
    # Save replayed CSV for validation plot overlay if needed
    replay_csv = os.path.join(bag_dir, f"{bag_name}_replay.csv")
    with open(replay_csv, 'w') as f:
        writer = csv.writer(f)
        writer.writerow(["pos_x", "pos_y", "pos_z"])
        for i in range(compare_len):
            writer.writerow([replayed_positions[i]['pos_x'], replayed_positions[i]['pos_y'], replayed_positions[i]['pos_z']])
    print(f"Saved replayed trajectory to {replay_csv}")
    
    if mse < 1e-4:
        print("\n[SUCCESS] Replay matches original trajectory with extremely high fidelity (MSE is negligible).")
        return True
    else:
        print("\n[WARNING] Trajectory mismatch detected. This might be due to chaotic divergence in physics simulation or time-step misalignment.")
        return False

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 replay_eval.py <bag_name_without_extension>")
        sys.exit(1)
    run_replay(sys.argv[1])
