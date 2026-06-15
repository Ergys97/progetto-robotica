import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def read_text(relative_path):
    return (ROOT / relative_path).read_text(encoding="utf-8")


class RunDemoScriptTest(unittest.TestCase):
    def test_script_exposes_flat_obstacle_headless_and_build_modes(self):
        script = read_text("scripts/run_demo.sh")

        self.assertIn("usage()", script)
        self.assertIn("flat)", script)
        self.assertIn("obstacle|obstacle_course)", script)
        self.assertIn("--headless", script)
        self.assertIn("--build", script)
        self.assertIn('scenario="obstacle_course"', script)
        self.assertIn('scenario:="${scenario}"', script)
        self.assertIn('headless="True"', script)
        self.assertIn('headless:="${headless}"', script)
        self.assertIn("colcon build --packages-select progetto_robotica", script)

    def test_script_sources_expected_ros_environment(self):
        script = read_text("scripts/run_demo.sh")

        self.assertIn("safe_source()", script)
        self.assertIn("set +u", script)
        self.assertIn("set -u", script)
        self.assertIn('ROS_DISTRO_NAME="${ROS_DISTRO_NAME:-jazzy}"', script)
        self.assertIn('ros_setup="/opt/ros/${ROS_DISTRO_NAME}/setup.bash"', script)
        self.assertIn("~/venv", script)
        self.assertIn('ROS_WS="${ROS_WS:-$HOME/ros2_ws}"', script)
        self.assertIn('install_setup="${ROS_WS}/install/setup.bash"', script)
        self.assertIn("ros2 launch progetto_robotica teleop_sim_launch.py", script)
        self.assertIn("http://localhost:5000", script)

    def test_readme_documents_short_demo_commands(self):
        readme = read_text("README.md")

        self.assertIn("bash scripts/run_demo.sh flat", readme)
        self.assertIn("bash scripts/run_demo.sh obstacle", readme)
        self.assertIn("bash scripts/run_demo.sh obstacle --headless", readme)
        self.assertIn("bash scripts/run_demo.sh flat --build", readme)


if __name__ == "__main__":
    unittest.main()
