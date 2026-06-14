import tempfile
from pathlib import Path
import unittest

from progetto_robotica.scene_builder import build_obstacle_scene, resolve_scene_xml


class SceneBuilderTest(unittest.TestCase):
    def test_flat_scenario_returns_original_xml(self):
        with self.subTest("flat returns input path"):
            with tempfile.TemporaryDirectory() as tmpdir:
                tmp_path = Path(tmpdir)
                source = tmp_path / "scene.xml"
                source.write_text("<mujoco><worldbody /></mujoco>", encoding="utf-8")

                result = resolve_scene_xml(str(source), "flat", str(tmp_path / "out"))

                self.assertEqual(result, str(source))

    def test_unknown_scenario_is_rejected(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            tmp_path = Path(tmpdir)
            source = tmp_path / "scene.xml"
            source.write_text("<mujoco><worldbody /></mujoco>", encoding="utf-8")

            with self.assertRaisesRegex(ValueError, "Unsupported scenario"):
                resolve_scene_xml(str(source), "stairs", str(tmp_path / "out"))

    def test_obstacle_scene_injects_expected_geoms_and_rewrites_include(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            tmp_path = Path(tmpdir)
            model = tmp_path / "g1.xml"
            model.write_text("<mujoco model='g1' />", encoding="utf-8")
            source = tmp_path / "scene.xml"
            source.write_text(
                "<mujoco><include file='g1.xml' /><worldbody><geom name='floor' type='plane' /></worldbody></mujoco>",
                encoding="utf-8",
            )

            generated = Path(build_obstacle_scene(str(source), str(tmp_path / "out")))
            text = generated.read_text(encoding="utf-8")

            self.assertEqual(generated.name, "obstacle_course_scene.xml")
            self.assertIn(f'file="{model.resolve().as_posix()}"', text)
            self.assertIn('name="obstacle_ramp"', text)
            self.assertIn('name="obstacle_step_1"', text)
            self.assertIn('name="obstacle_step_2"', text)
            self.assertIn('name="obstacle_step_3"', text)


if __name__ == "__main__":
    unittest.main()
