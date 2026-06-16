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
        self.assertIn("python3 -m colcon build --packages-select progetto_robotica", script)

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

    def test_readme_setup_uses_real_clone_url_before_requirements(self):
        readme = read_text("README.md")

        clone_cmd = "git clone https://github.com/Ergys97/progetto-robotica.git"
        pip_cmd = "~/venv/bin/pip install -r requirements.txt"

        self.assertIn(clone_cmd, readme)
        self.assertNotIn("git clone <questo-repo>", readme)
        self.assertLess(readme.index(clone_cmd), readme.index(pip_cmd))

    def test_readme_documents_venv_build_and_streaming_dashboard(self):
        readme = read_text("README.md")

        self.assertIn("python3 -m colcon build", readme)
        self.assertIn("Plotly streaming", readme)
        self.assertIn("Ctrl+Shift+R", readme)


if __name__ == "__main__":
    unittest.main()
