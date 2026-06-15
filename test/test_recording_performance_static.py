import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def read_text(relative_path):
    return (ROOT / relative_path).read_text(encoding="utf-8")


class RecordingPerformanceStaticTest(unittest.TestCase):
    def test_launch_exposes_runtime_rate_controls(self):
        launch = read_text("launch/teleop_sim_launch.py")

        self.assertIn("DeclareLaunchArgument('telemetry_hz'", launch)
        self.assertIn("DeclareLaunchArgument('csv_hz'", launch)
        self.assertIn("DeclareLaunchArgument('viewer_fps'", launch)
        self.assertIn("DeclareLaunchArgument('record_profile'", launch)
        self.assertIn("'telemetry_hz': LaunchConfiguration('telemetry_hz')", launch)
        self.assertIn("'csv_hz': LaunchConfiguration('csv_hz')", launch)
        self.assertIn("'viewer_fps': LaunchConfiguration('viewer_fps')", launch)
        self.assertIn("'record_profile': LaunchConfiguration('record_profile')", launch)

    def test_mujoco_loop_decimates_publication_csv_and_viewer(self):
        sim = read_text("progetto_robotica/mujoco_sim.py")

        self.assertIn("self.telemetry_step_interval", sim)
        self.assertIn("self.csv_step_interval", sim)
        self.assertIn("self.viewer_step_interval", sim)
        self.assertIn("self.counter % self.csv_step_interval == 0", sim)
        self.assertIn("self.counter % self.telemetry_step_interval == 0", sim)
        self.assertIn("self.counter % self.viewer_step_interval == 0", sim)
        self.assertNotIn("if self.is_logging_csv:\n            self._write_csv_row()", sim)

    def test_web_recording_uses_lightweight_profiles_and_mcap(self):
        web_node = read_text("progetto_robotica/web_teleop.py")
        constants = read_text("progetto_robotica/constants.py")

        self.assertIn("RECORDING_TOPIC_PROFILES", web_node)
        self.assertIn("RECORDING_TOPIC_PROFILES", constants)
        self.assertIn("self.declare_parameter('record_profile', 'metrics')", web_node)
        self.assertIn('"--storage", "mcap"', web_node)
        self.assertIn("topics = constants.RECORDING_TOPIC_PROFILES.get(", web_node)
        self.assertIn("*topics", web_node)

    def test_web_auto_saves_dashboard_metrics(self):
        web_node = read_text("progetto_robotica/web_teleop.py")
        dashboard = read_text("web/static/js/dashboard.js")

        self.assertIn("@app.route('/api/metrics/save'", web_node)
        self.assertIn("def save_metrics", web_node)
        self.assertIn("metrics_dir = os.path.join(self.bag_dir, 'metrics')", web_node)
        self.assertIn("saveMetricsToBackend(summary)", dashboard)
        self.assertIn("fetch('/api/metrics/save'", dashboard)

    def test_readme_documents_recording_profiles_and_rate_controls(self):
        readme = read_text("README.md")

        self.assertIn("record_profile:=metrics", readme)
        self.assertIn("telemetry_hz:=50.0", readme)
        self.assertIn("csv_hz:=50.0", readme)
        self.assertIn("viewer_fps:=15.0", readme)


if __name__ == "__main__":
    unittest.main()
