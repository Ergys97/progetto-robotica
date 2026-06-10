"""Pure-math helpers shared by mujoco_sim and replay_eval (no ROS deps)."""
import numpy as np


def get_gravity_orientation(quaternion):
    """Gravity vector projected in body frame. quaternion = [qw, qx, qy, qz]."""
    qw, qx, qy, qz = quaternion[0], quaternion[1], quaternion[2], quaternion[3]
    g = np.zeros(3)
    g[0] = 2 * (-qz * qx + qw * qy)
    g[1] = -2 * (qz * qy + qw * qx)
    g[2] = 1 - 2 * (qw * qw + qz * qz)
    return g


def pd_control(target_q, q, kp, target_dq, dq, kd):
    return (target_q - q) * kp + (target_dq - dq) * kd


def quat_to_roll_pitch(qw, qx, qy, qz):
    sinr_cosp = 2 * (qw * qx + qy * qz)
    cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
    roll = np.arctan2(sinr_cosp, cosr_cosp)
    sinp = 2 * (qw * qy - qz * qx)
    pitch = np.arcsin(np.clip(sinp, -1.0, 1.0))
    return roll, pitch


def is_fallen(qw, qx, qy, qz, threshold_deg):
    roll, pitch = quat_to_roll_pitch(qw, qx, qy, qz)
    t = np.deg2rad(threshold_deg)
    return bool(abs(roll) > t or abs(pitch) > t)


def find_log_index(sim_times, t):
    """Index of the last logged row with sim_time <= t (clamped to valid range)."""
    i = int(np.searchsorted(sim_times, t, side='right')) - 1
    return max(0, min(i, len(sim_times) - 1))
