import numpy as np
import pytest

from progetto_robotica.sim_utils import (
    get_gravity_orientation, pd_control, quat_to_roll_pitch,
    is_fallen, find_log_index,
)


def test_gravity_identity():
    # robot dritto: gravità proiettata = [0, 0, -1]
    g = get_gravity_orientation(np.array([1.0, 0.0, 0.0, 0.0]))
    np.testing.assert_allclose(g, [0.0, 0.0, -1.0], atol=1e-9)


def test_gravity_roll_90():
    c = np.cos(np.pi / 4)
    s = np.sin(np.pi / 4)
    g = get_gravity_orientation(np.array([c, s, 0.0, 0.0]))  # roll +90°
    np.testing.assert_allclose(g, [0.0, -1.0, 0.0], atol=1e-9)


def test_pd_control():
    tau = pd_control(np.array([1.0]), np.array([0.0]), np.array([2.0]),
                     np.array([0.0]), np.array([0.5]), np.array([4.0]))
    np.testing.assert_allclose(tau, [2.0 - 2.0], atol=1e-9)  # 2*(1-0) + 4*(0-0.5)


def test_roll_pitch_identity():
    roll, pitch = quat_to_roll_pitch(1.0, 0.0, 0.0, 0.0)
    assert abs(roll) < 1e-9 and abs(pitch) < 1e-9


def test_is_fallen_threshold():
    # roll = 40° > soglia 35° -> caduto
    half = np.deg2rad(40.0) / 2
    assert is_fallen(np.cos(half), np.sin(half), 0.0, 0.0, 35.0) is True
    # roll = 30° < soglia -> ok
    half = np.deg2rad(30.0) / 2
    assert is_fallen(np.cos(half), np.sin(half), 0.0, 0.0, 35.0) is False


def test_find_log_index():
    times = np.array([0.0, 0.02, 0.04])
    assert find_log_index(times, 0.0) == 0
    assert find_log_index(times, 0.019) == 0
    assert find_log_index(times, 0.02) == 1
    assert find_log_index(times, 99.0) == 2
    assert find_log_index(times, -1.0) == 0
