import csv
import os
import numpy as np
import torch
import yaml
import mujoco

from progetto_robotica import sim_utils

DEFAULT_RL_GYM = os.path.expanduser('~/unitree_rl_gym')
DEFAULT_BAG_DIR = os.path.expanduser('~/progetto_robotica_bags')

def check_policy(bag_name):
    csv_path = os.path.join(DEFAULT_BAG_DIR, f"{bag_name}.csv")
    with open(csv_path, 'r') as f:
        reader = csv.DictReader(f)
        rows = list(reader)

    config_path = os.path.join(DEFAULT_RL_GYM, 'deploy/deploy_mujoco/configs/g1.yaml')
    policy_path = os.path.join(DEFAULT_RL_GYM, 'deploy/pre_train/g1/motion.pt')

    with open(config_path, 'r') as f:
        config = yaml.load(f, Loader=yaml.FullLoader)

    default_angles = np.array(config["default_angles"], dtype=np.float32)
    ang_vel_scale = config["ang_vel_scale"]
    dof_pos_scale = config["dof_pos_scale"]
    dof_vel_scale = config["dof_vel_scale"]
    cmd_scale = np.array(config["cmd_scale"], dtype=np.float32)
    num_actions = config["num_actions"]
    num_obs = config["num_obs"]

    policy = torch.jit.load(policy_path)

    # 1. Reconstruct observation from logged row at 0.02s
    row = rows[1] # sim_time = 0.02
    qpos_keys = sorted((k for k in row if k.startswith('qpos_')), key=lambda x: int(x.split('_')[1]))
    qvel_keys = sorted((k for k in row if k.startswith('qvel_')), key=lambda x: int(x.split('_')[1]))

    qpos = np.array([float(row[k]) for k in qpos_keys])
    qvel = np.array([float(row[k]) for k in qvel_keys])
    cmd = np.array([float(row['cmd_vx']), float(row['cmd_vy']), float(row['cmd_wz'])], dtype=np.float32)

    omega = qvel[3:6] * ang_vel_scale
    gravity_orientation = sim_utils.get_gravity_orientation(qpos[3:7])
    qj = (qpos[7:] - default_angles) * dof_pos_scale
    dqj = qvel[6:] * dof_vel_scale
    
    # In live run at counter = 10, action is all 0
    action_prev = np.zeros(num_actions, dtype=np.float32)
    
    period = 0.8
    phase = 0.02 % period / period
    
    obs_log = np.zeros(num_obs, dtype=np.float32)
    obs_log[:3] = omega
    obs_log[3:6] = gravity_orientation
    obs_log[6:9] = cmd * cmd_scale
    obs_log[9:9 + num_actions] = qj
    obs_log[9 + num_actions:9 + 2 * num_actions] = dqj
    obs_log[9 + 2 * num_actions:9 + 3 * num_actions] = action_prev
    obs_log[9 + 3 * num_actions:9 + 3 * num_actions + 2] = np.array([np.sin(2 * np.pi * phase), np.cos(2 * np.pi * phase)])

    with torch.no_grad():
        action_log = policy(torch.from_numpy(obs_log).unsqueeze(0)).numpy().squeeze()

    print("Logged Observation components:")
    print("  omega:", omega)
    print("  gravity:", gravity_orientation)
    print("  cmd:", cmd)
    print("  qj:", qj)
    print("  dqj:", dqj)
    print("  phase:", [np.sin(2 * np.pi * phase), np.cos(2 * np.pi * phase)])
    print("Logged Policy output action:")
    print("  action:", action_log)

if __name__ == '__main__':
    check_policy('bag_20260611_013845')
