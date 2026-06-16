import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def read_text(relative_path):
    return (ROOT / relative_path).read_text(encoding="utf-8")


class ReportStaticTest(unittest.TestCase):
    def test_report_documents_obstacle_transient_and_replay_trajectory(self):
        report = read_text("docs/report/relazione.md")

        self.assertIn("minimo transitorio a 3 Hz", report)
        self.assertIn("![Traiettoria flat per replay](figures/traiettoria_flat.png)", report)

    def test_figure_generator_uses_readable_architecture_topic_bus(self):
        figures = read_text("scripts/generate_report_figures.py")

        self.assertIn("Topic ROS 2", figures)
        self.assertIn("draw_elbow_arrow", figures)
        self.assertIn("draw_box(ax, (0.62, 0.42)", figures)
        self.assertNotIn('"/imu  /joint_states  /odom', figures)

    def test_pdf_includes_transient_note_and_trajectory_figure(self):
        pdf_builder = read_text("scripts/build_report_pdf.py")

        self.assertIn("minimo transitorio a 3 Hz", pdf_builder)
        self.assertIn("traiettoria_flat.png", pdf_builder)
        self.assertIn("Figura 5", pdf_builder)

    def test_pdf_builder_uses_required_page_formatting(self):
        pdf_builder = read_text("scripts/build_report_pdf.py")

        self.assertIn("MARGIN_CM = 2.0", pdf_builder)
        self.assertIn("BODY_FONT_SIZE = 11.0", pdf_builder)
        self.assertIn("LINE_SPACING = 1.08", pdf_builder)
        self.assertIn("BODY_LEADING = BODY_FONT_SIZE * LINE_SPACING / 72.0", pdf_builder)

    def test_pdf_uses_root_architecture_image_requested_for_submission(self):
        pdf_builder = read_text("scripts/build_report_pdf.py")

        self.assertIn('ROOT / "architettura_ros2.png"', pdf_builder)
        self.assertIn("ARCHITECTURE_IMAGE", pdf_builder)
        self.assertIn("ARCHITECTURE_IMAGE,", pdf_builder)


if __name__ == "__main__":
    unittest.main()
