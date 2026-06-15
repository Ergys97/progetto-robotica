import unittest

import numpy as np

from progetto_robotica.sim_utils import (
    detect_foot_contacts,
    find_log_index,
    get_gravity_orientation,
    is_fallen,
    pd_control,
    quat_to_roll_pitch,
    rate_to_step_interval,
)


class SimUtilsTest(unittest.TestCase):
    def test_gravity_identity(self):
        g = get_gravity_orientation(np.array([1.0, 0.0, 0.0, 0.0]))
        np.testing.assert_allclose(g, [0.0, 0.0, -1.0], atol=1e-9)

    def test_gravity_roll_90(self):
        c = np.cos(np.pi / 4)
        s = np.sin(np.pi / 4)
        g = get_gravity_orientation(np.array([c, s, 0.0, 0.0]))
        np.testing.assert_allclose(g, [0.0, -1.0, 0.0], atol=1e-9)

    def test_pd_control(self):
        tau = pd_control(
            np.array([1.0]),
            np.array([0.0]),
            np.array([2.0]),
            np.array([0.0]),
            np.array([0.5]),
            np.array([4.0]),
        )
        np.testing.assert_allclose(tau, [0.0], atol=1e-9)

    def test_roll_pitch_identity(self):
        roll, pitch = quat_to_roll_pitch(1.0, 0.0, 0.0, 0.0)
        self.assertLess(abs(roll), 1e-9)
        self.assertLess(abs(pitch), 1e-9)

    def test_is_fallen_threshold(self):
        half = np.deg2rad(40.0) / 2
        self.assertIs(is_fallen(np.cos(half), np.sin(half), 0.0, 0.0, 35.0), True)

        half = np.deg2rad(30.0) / 2
        self.assertIs(is_fallen(np.cos(half), np.sin(half), 0.0, 0.0, 35.0), False)

    def test_find_log_index(self):
        times = np.array([0.0, 0.02, 0.04])
        self.assertEqual(find_log_index(times, 0.0), 0)
        self.assertEqual(find_log_index(times, 0.019), 0)
        self.assertEqual(find_log_index(times, 0.02), 1)
        self.assertEqual(find_log_index(times, 99.0), 2)
        self.assertEqual(find_log_index(times, -1.0), 0)

    def test_rate_to_step_interval(self):
        self.assertEqual(rate_to_step_interval(0.002, 50), 10)
        self.assertEqual(rate_to_step_interval(0.002, 33), 15)
        self.assertEqual(rate_to_step_interval(0.002, 0), 1)
        self.assertEqual(rate_to_step_interval(0.002, -1), 1)

    def test_detect_foot_contacts_counts_obstacles_as_support(self):
        contacts = [(10, 99), (42, 11)]
        left, right = detect_foot_contacts(
            contacts,
            support_geom_ids={99},
            left_foot_geom_ids={10},
            right_foot_geom_ids={11},
        )

        self.assertIs(left, True)
        self.assertIs(right, False)

    def test_detect_foot_contacts_counts_reversed_geom_order(self):
        contacts = [(99, 11)]
        left, right = detect_foot_contacts(
            contacts,
            support_geom_ids={99},
            left_foot_geom_ids={10},
            right_foot_geom_ids={11},
        )

        self.assertIs(left, False)
        self.assertIs(right, True)

    def test_detect_foot_contacts_ignores_non_support_collisions(self):
        contacts = [(10, 42), (11, 43)]
        left, right = detect_foot_contacts(
            contacts,
            support_geom_ids={99},
            left_foot_geom_ids={10},
            right_foot_geom_ids={11},
        )

        self.assertIs(left, False)
        self.assertIs(right, False)


if __name__ == "__main__":
    unittest.main()
