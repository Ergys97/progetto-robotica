import csv
import os
import numpy as np
import torch
import yaml
import mujoco

from progetto_robotica import sim_utils

DEFAULT_RL_GYM = os.path.expanduser('~/unitree_rl_gym')
DEFAULT_BAG_DIR = os.path.expanduser('~/progetto_robotica_bags')

def compare(bag_name):
    csv_path = os.path.join(DEFAULT_BAG_DIR, f"{bag_name}.csv")
    with open(csv_path, 'r') as f:
        reader = csv.DictReader(f)
        rows = list(reader)

    sim_times = np.array([float(r['sim_time']) for r in rows])
    cmds = np.array([[float(r['cmd_vx']), float(r['cmd_vy']), float(r['cmd_wz'])] for r in rows], dtype=np.float32)

    config_path = os.path.join(DEFAULT_RL_GYM, 'deploy/deploy_mujoco/configs/g1.yaml')
    xml_path = os.path.join(DEFAULT_RL_GYM, 'resources/robots/g1_description/scene.xml')
    policy_path = os.path.join(DEFAULT_RL_GYM, 'deploy/pre_train/g1/motion.pt')

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

    m = mujoco.MjModel.from_xml_path(xml_path)
    d = mujoco.MjData(m)
    m.opt.timestep = simulation_dt
    policy = torch.jit.load(policy_path)

    mujoco.mj_resetData(m, d)
    qpos_keys = sorted((k for k in rows[0] if k.startswith('qpos_')), key=lambda x: int(x.split('_')[1]))
    qvel_keys = sorted((k for k in rows[0] if k.startswith('qvel_')), key=lambda x: int(x.split('_')[1]))
    d.qpos[:] = [float(rows[0][k]) for k in qpos_keys]
    d.qvel[:] = [float(rows[0][k]) for k in qvel_keys]

    cmd = np.zeros(3, dtype=np.float32)
    action = np.zeros(num_actions, dtype=np.float32)
    target_dof_pos = default_angles.copy()
    obs = np.zeros(num_obs, dtype=np.float32)

    # Let's compare step-by-step
    print("Step-by-step detail (0 to 20):")
    counter = 0
    for step in range(25):
        # 1. Compute torque
        tau = sim_utils.pd_control(target_dof_pos, d.qpos[7:], kps, np.zeros_like(kds), d.qvel[6:], kds)
        
        # Look up logged row at this step index
        log_row = rows[step]
        log_qpos = np.array([float(log_row[k]) for k in qpos_keys])
        log_qvel = np.array([float(log_row[k]) for k in qvel_keys])
        
        qpos_diff = np.max(np.abs(d.qpos - log_qpos))
        qvel_diff = np.max(np.abs(d.qvel - log_qvel))
        
        print(f"Step {step:2d} (Time {float(log_row['sim_time']):.3f}s) | qpos diff: {qpos_diff:.2e} | qvel diff: {qvel_diff:.2e}")
        if qpos_diff > 1e-6 or qvel_diff > 1e-6:
            target_keys = sorted((k for k in rows[0] if k.startswith('target_')), key=lambda x: int(x.split('_')[1]))
            log_targets = np.array([float(log_row[k]) for k in target_keys]) if target_keys else None
            print("  Replay qpos[7]:", d.qpos[7])
            print("  Log qpos[7]   :", log_qpos[7])
            print("  Replay qvel[6]:", d.qvel[6])
            print("  Log qvel[6]   :", log_qvel[6])
            print("  Replay target_dof_pos:", target_dof_pos)
            if log_targets is not None:
                print("  Log target_dof_pos   :", log_targets)
                print("  Target diff          :", np.max(np.abs(target_dof_pos - log_targets)))
            break
            
        d.ctrl[:] = tau
        mujoco.mj_step(m, d)
        counter += 1
        
        if counter % control_decimation == 0:
            sim_time = counter * simulation_dt
            log_idx = step + 1 # since we log at every step, the row index corresponds to step + 1
            cmd[:] = cmds[log_idx]
            qj = (d.qpos[7:] - default_angles) * dof_pos_scale
            dqj = d.qvel[6:] * dof_vel_scale
            gravity_orientation = sim_utils.get_gravity_orientation(d.qpos[3:7])
            omega = d.qvel[3:6] * ang_vel_scale

            period = 0.8
            phase = sim_time % period / period
            n = num_actions
            obs[:3] = omega
            obs[3:6] = gravity_orientation
            obs[6:9] = cmd * cmd_scale
            obs[9:9 + n] = qj
            obs[9 + n:9 + 2 * n] = dqj
            obs[9 + 2 * n:9 + 3 * n] = action
            obs[9 + 3 * n:9 + 3 * n + 2] = np.array([np.sin(2 * np.pi * phase), np.cos(2 * np.pi * phase)])
            
            with torch.no_grad():
                action[:] = policy(torch.from_numpy(obs).unsqueeze(0)).numpy().squeeze()
            target_dof_pos[:] = action * action_scale + default_angles
            print("Observation vector at step 10:")
            print("  omega:", obs[:3])
            print("  gravity:", obs[3:6])
            print("  cmd:", obs[6:9])
            print("  qj:", obs[9:21])
            print("  dqj:", obs[21:33])
            print("  action (prev):", obs[33:45])
            print("  phase:", obs[45:47])
            print("Output action:")
            print("  action:", action)
            print("Output target_dof_pos:")
            print("  target_dof_pos:", target_dof_pos)

if __name__ == '__main__':
    compare('bag_20260611_015233')
