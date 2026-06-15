import re
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def read_text(relative_path):
    return (ROOT / relative_path).read_text(encoding="utf-8")


class FrontendStaticTest(unittest.TestCase):
    def test_dashboard_copy_is_consistently_italian_and_scientific(self):
        html = read_text("web/templates/index.html")
        js = read_text("web/static/js/dashboard.js")

        self.assertIn("Protocollo sperimentale", html)
        self.assertIn("Metriche principali", html)
        self.assertIn("Disconnesso", js)
        self.assertNotIn("Disconnected", js)
        self.assertNotRegex(html, r"Sidebar|Dashboard Grid|Touch buttons")
        self.assertNotRegex(js, r"Slidind|glows")

    def test_cards_use_compact_radius_and_no_negative_tracking(self):
        css = read_text("web/static/css/style.css")

        card_block = re.search(r"\.card\s*\{(?P<body>.*?)\}", css, re.S)
        self.assertIsNotNone(card_block)
        self.assertIn("border-radius: 8px", card_block.group("body"))
        self.assertNotIn("letter-spacing: -", css)

    def test_metrics_protocol_declares_replay_scope(self):
        metrics_doc = read_text("docs/metrics/protocollo-metriche.md")

        self.assertIn("La fedelta di replay viene misurata sullo scenario `flat`", metrics_doc)
        self.assertIn("Lo scenario `obstacle_course`", metrics_doc)

    def test_dashboard_exports_session_metrics(self):
        html = read_text("web/templates/index.html")
        js = read_text("web/static/js/dashboard.js")

        self.assertIn("Scarica JSON", html)
        self.assertIn("Scarica CSV", html)
        self.assertIn('id="btn-download-json"', html)
        self.assertIn('id="btn-download-csv"', html)
        self.assertIn("sessionMetrics", js)
        self.assertIn("downloadMetricsJson", js)
        self.assertIn("downloadMetricsCsv", js)

    def test_latency_chart_is_present(self):
        html = read_text("web/templates/index.html")
        js = read_text("web/static/js/dashboard.js")

        self.assertIn('id="plot-latency"', html)
        self.assertIn("Latenza comando nel tempo", html)
        self.assertIn("latencyData", js)
        self.assertIn("latencyPlotDiv", js)

    def test_fall_alarm_is_non_blocking(self):
        html = read_text("web/templates/index.html")
        css = read_text("web/static/css/style.css")

        self.assertIn('id="fall-alarm"', html)
        self.assertIn('id="btn-record-stop"', html)
        self.assertIn('id="btn-reset"', html)
        triggered_block = re.search(r"\.alarm-overlay\.triggered\s*\{(?P<body>.*?)\}", css, re.S)
        self.assertIsNotNone(triggered_block)
        self.assertIn("pointer-events: none", triggered_block.group("body"))
        self.assertNotIn("pointer-events: auto", triggered_block.group("body"))

    def test_plotly_charts_are_throttled(self):
        js = read_text("web/static/js/dashboard.js")

        self.assertIn("const chartUpdateIntervalMs = 200", js)
        self.assertIn("let lastChartUpdateMs = 0", js)
        self.assertIn("function updateCharts()", js)
        self.assertIn("if (now - lastChartUpdateMs >= chartUpdateIntervalMs)", js)
        self.assertIn("updateCharts();", js)

    def test_web_status_exposes_scenario(self):
        launch = read_text("launch/teleop_sim_launch.py")
        web_node = read_text("progetto_robotica/web_teleop.py")

        self.assertIn("'scenario': LaunchConfiguration('scenario')", launch)
        self.assertIn("self.declare_parameter('scenario', 'flat')", web_node)
        self.assertIn("'scenario': ros_node.scenario", web_node)


if __name__ == "__main__":
    unittest.main()
