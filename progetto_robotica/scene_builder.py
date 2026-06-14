"""Helpers for generating MuJoCo scenario XML files without modifying vendor assets."""

from __future__ import annotations

import os
import xml.etree.ElementTree as ET
from pathlib import Path


SUPPORTED_SCENARIOS = {"flat", "obstacle_course"}


def resolve_scene_xml(xml_path: str, scenario: str, output_dir: str) -> str:
    """Return the XML path that MuJoCo should load for the requested scenario."""
    if scenario not in SUPPORTED_SCENARIOS:
        supported = ", ".join(sorted(SUPPORTED_SCENARIOS))
        raise ValueError(f"Unsupported scenario '{scenario}'. Supported scenarios: {supported}")
    if scenario == "flat":
        return xml_path
    return build_obstacle_scene(xml_path, output_dir)


def build_obstacle_scene(base_xml_path: str, output_dir: str) -> str:
    """Generate an obstacle-course scene from a base MuJoCo XML scene."""
    base_path = Path(os.path.expanduser(base_xml_path)).resolve()
    tree = ET.parse(base_path)
    root = tree.getroot()

    _prepare_xml_for_relocation(root, base_path.parent)
    worldbody = root.find("worldbody")
    if worldbody is None:
        worldbody = ET.SubElement(root, "worldbody")

    _remove_existing_generated_obstacles(worldbody)
    _append_obstacles(worldbody)

    target_dir = Path(os.path.expanduser(output_dir)).resolve()
    target_dir.mkdir(parents=True, exist_ok=True)
    generated_path = target_dir / "obstacle_course_scene.xml"
    tree.write(generated_path, encoding="utf-8", xml_declaration=True)
    return str(generated_path)


def _prepare_xml_for_relocation(root: ET.Element, base_dir: Path) -> None:
    _make_compiler_asset_dirs_absolute(root, base_dir)
    _expand_includes(root, base_dir)


def _make_compiler_asset_dirs_absolute(root: ET.Element, base_dir: Path) -> None:
    for compiler in root.findall(".//compiler"):
        for attr in ("meshdir", "texturedir"):
            value = compiler.get(attr)
            if not value:
                continue
            path = Path(os.path.expanduser(value))
            if not path.is_absolute():
                compiler.set(attr, (base_dir / path).resolve().as_posix())


def _expand_includes(parent: ET.Element, base_dir: Path) -> None:
    index = 0
    while index < len(parent):
        child = parent[index]
        if child.tag != "include":
            _expand_includes(child, base_dir)
            index += 1
            continue

        include_file = child.get("file")
        if not include_file:
            parent.remove(child)
            continue

        include_path = Path(os.path.expanduser(include_file))
        if not include_path.is_absolute():
            include_path = (base_dir / include_path).resolve()

        included_tree = ET.parse(include_path)
        included_root = included_tree.getroot()
        _prepare_xml_for_relocation(included_root, include_path.parent)

        parent.remove(child)
        for included_child in list(included_root):
            parent.insert(index, included_child)
            index += 1


def _remove_existing_generated_obstacles(worldbody: ET.Element) -> None:
    generated_names = {"obstacle_ramp", "obstacle_step_1", "obstacle_step_2", "obstacle_step_3"}
    for geom in list(worldbody.findall("geom")):
        if geom.get("name") in generated_names:
            worldbody.remove(geom)


def _append_obstacles(worldbody: ET.Element) -> None:
    ET.SubElement(
        worldbody,
        "geom",
        {
            "name": "obstacle_ramp",
            "type": "box",
            "size": "0.70 0.45 0.035",
            "pos": "2.0 0 0.035",
            "euler": "0 0.139626 0",
            "rgba": "0.45 0.48 0.52 1",
            "friction": "1.0 0.005 0.0001",
        },
    )
    for index, (x_pos, height) in enumerate(((3.0, 0.02), (3.35, 0.03), (3.70, 0.04)), start=1):
        ET.SubElement(
            worldbody,
            "geom",
            {
                "name": f"obstacle_step_{index}",
                "type": "box",
                "size": f"0.13 0.45 {height / 2:.3f}",
                "pos": f"{x_pos:.2f} 0 {height / 2:.3f}",
                "rgba": "0.36 0.39 0.43 1",
                "friction": "1.0 0.005 0.0001",
            },
        )
