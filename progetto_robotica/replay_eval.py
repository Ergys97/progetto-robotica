#!/usr/bin/env python3
"""Replay deterministico di una sessione registrata e calcolo MSE vs traiettoria live.

Uso: ros2 run progetto_robotica replay_eval <bag_name> [--bag-dir DIR] [--rl-gym DIR]
"""
import argparse
import csv
import os

import numpy as np
import torch
import yaml
import mujoco

from progetto_robotica import sim_utils

DEFAULT_RL_GYM = os.path.expanduser(os.environ.get('UNITREE_RL_GYM_PATH', '~/unitree_rl_gym'))
DEFAULT_BAG_DIR = os.path.expanduser('~/progetto_robotica_bags')


def run_replay(bag_name, bag_dir=DEFAULT_BAG_DIR, rl_gym=DEFAULT_RL_GYM):
    csv_path = os.path.join(bag_dir, f"{bag_name}.csv")
    if not os.path.exists(csv_path):
        print(f"Error: log file not found at {csv_path}")
        return False

    with open(csv_path, 'r') as f:
        reader = csv.DictReader(f)
        rows = list(reader)
    if not rows:
        print("Error: log file is empty.")
        return False

    sim_times = np.array([float(r['sim_time']) for r in rows])
    cmds = np.array([[float(r['cmd_vx']), float(r['cmd_vy']), float(r['cmd_wz'])] for r in rows],
                    dtype=np.float32)
    live_pos = np.array([[float(r['pos_x']), float(r['pos_y']), float(r['pos_z'])] for r in rows])
    print(f"Loaded {len(rows)} telemetry frames ({sim_times[-1]:.1f}s of sim time).")

    config_path = os.path.join(rl_gym, 'deploy/deploy_mujoco/configs/g1.yaml')
    xml_path = os.path.join(rl_gym, 'resources/robots/g1_description/scene.xml')
    policy_path = os.path.join(rl_gym, 'deploy/pre_train/g1/motion.pt')

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

    # Stato iniziale: identico al reset live (riga 0 del CSV = stato post-reset)
    mujoco.mj_resetData(m, d)
    qpos_keys = sorted((k for k in rows[0] if k.startswith('qpos_')), key=lambda x: int(x.split('_')[1]))
    qvel_keys = sorted((k for k in rows[0] if k.startswith('qvel_')), key=lambda x: int(x.split('_')[1]))
    target_keys = sorted((k for k in rows[0] if k.startswith('target_')), key=lambda x: int(x.split('_')[1]))
    use_logged_targets = len(target_keys) == num_actions
    d.qpos[:] = [float(rows[0][k]) for k in qpos_keys]
    d.qvel[:] = [float(rows[0][k]) for k in qvel_keys]

    cmd = np.zeros(3, dtype=np.float32)
    action = np.zeros(num_actions, dtype=np.float32)
    target_dof_pos = default_angles.copy()
    obs = np.zeros(num_obs, dtype=np.float32)

    total_steps = int(round(sim_times[-1] / simulation_dt))
    replayed = [d.qpos[0:3].copy()]  # riga 0: stato iniziale, come nel CSV live

    counter = 0
    for _ in range(total_steps):
        tau = sim_utils.pd_control(target_dof_pos, d.qpos[7:], kps,
                                   np.zeros_like(kds), d.qvel[6:], kds)
        d.ctrl[:] = tau
        mujoco.mj_step(m, d)

        counter += 1
        if counter % control_decimation == 0:
            sim_time = counter * simulation_dt
            log_idx = sim_utils.find_log_index(sim_times, sim_time)
            if use_logged_targets:
                target_dof_pos[:] = [float(rows[log_idx][k]) for k in target_keys]
            else:
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
                obs[9 + 3 * n:9 + 3 * n + 2] = np.array([np.sin(2 * np.pi * phase),
                                                          np.cos(2 * np.pi * phase)])
                with torch.no_grad():
                    action[:] = policy(torch.from_numpy(obs).unsqueeze(0)).numpy().squeeze()
                target_dof_pos[:] = action * action_scale + default_angles

            replayed.append(d.qpos[0:3].copy())

    replayed = np.array(replayed)
    compare_len = min(len(live_pos), len(replayed))
    sq_err = np.sum((live_pos[:compare_len] - replayed[:compare_len]) ** 2, axis=1)
    mse = float(np.mean(sq_err))
    rmse = float(np.sqrt(mse))

    print("\n--- Replay Evaluation Results ---")
    print(f"Compared steps: {compare_len}")
    print(f"MSE:  {mse:.8f} m^2")
    print(f"RMSE: {rmse:.6f} m")
    print(f"Max deviation: {np.sqrt(np.max(sq_err)):.6f} m")

    out_csv = os.path.join(bag_dir, f"{bag_name}_replay.csv")
    with open(out_csv, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(["pos_x", "pos_y", "pos_z"])
        w.writerows(replayed[:compare_len].tolist())
    print(f"Saved replayed trajectory to {out_csv}")

    if mse < 1e-4:
        print("\n[SUCCESS] Replay matches live trajectory (MSE negligible).")
        return True
    print("\n[WARNING] Trajectory mismatch: check that recording started from reset state.")
    return False


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag_name')
    parser.add_argument('--bag-dir', default=DEFAULT_BAG_DIR)
    parser.add_argument('--rl-gym', default=DEFAULT_RL_GYM)
    ns = parser.parse_args(args)
    ok = run_replay(ns.bag_name, os.path.expanduser(ns.bag_dir), os.path.expanduser(ns.rl_gym))
    raise SystemExit(0 if ok else 1)


if __name__ == '__main__':
    main()
