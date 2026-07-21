#!/usr/bin/env python3

import argparse
import copy
from pathlib import Path
import xml.etree.ElementTree as ET


def remove_children(parent: ET.Element, names: set[str]) -> None:
    for child in list(parent):
        if child.tag in names:
            parent.remove(child)


def remove_tag(parent: ET.Element, tag: str) -> None:
    for child in list(parent.findall(tag)):
        parent.remove(child)


def add_basic_material(
    material: ET.Element,
    ambient: str,
    diffuse: str,
    specular: str = "0.08 0.08 0.08 1",
) -> None:
    ET.SubElement(material, "ambient").text = ambient
    ET.SubElement(material, "diffuse").text = diffuse
    ET.SubElement(material, "specular").text = specular


def convert_script_materials(world: ET.Element) -> None:
    for material in world.findall(".//material"):
        script = material.find("script")
        if script is None:
            continue

        script_name = (script.findtext("name") or "").strip()

        remove_tag(material, "script")
        remove_tag(material, "ambient")
        remove_tag(material, "diffuse")
        remove_tag(material, "specular")
        remove_tag(material, "pbr")

        if script_name == "vrc/grey_wall":
            add_basic_material(
                material,
                "0.9 0.9 0.9 1",
                "0.9 0.9 0.9 1",
            )

            pbr = ET.SubElement(material, "pbr")
            metal = ET.SubElement(pbr, "metal")
            ET.SubElement(
                metal,
                "albedo_map",
            ).text = "model://grey_wall/materials/textures/grey_wall.png"
            ET.SubElement(metal, "metalness").text = "0.0"
            ET.SubElement(metal, "roughness").text = "0.85"

        elif script_name == "Gazebo/Wood":
            add_basic_material(
                material,
                "0.34 0.21 0.11 1",
                "0.46 0.29 0.15 1",
            )

        elif script_name == "Gazebo/Grey":
            add_basic_material(
                material,
                "0.48 0.48 0.48 1",
                "0.58 0.58 0.58 1",
            )

        else:
            add_basic_material(
                material,
                "0.52 0.52 0.52 1",
                "0.62 0.62 0.62 1",
            )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--base", required=True)
    parser.add_argument("--legacy", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()

    base_path = Path(args.base).expanduser().resolve()
    legacy_path = Path(args.legacy).expanduser().resolve()
    output_path = Path(args.output).expanduser().resolve()

    if not base_path.is_file():
        raise FileNotFoundError(f"PX4 基础 world 不存在：{base_path}")

    if not legacy_path.is_file():
        raise FileNotFoundError(f"旧 Student Center world 不存在：{legacy_path}")

    base_tree = ET.parse(base_path)
    legacy_tree = ET.parse(legacy_path)

    base_root = base_tree.getroot()
    legacy_root = legacy_tree.getroot()

    base_world = base_root.find("world")
    legacy_world = legacy_root.find("world")

    if base_world is None:
        raise RuntimeError("PX4 基础 world 缺少 <sdf><world> 结构")

    if legacy_world is None:
        raise RuntimeError("旧 Student Center world 缺少 <sdf><world> 结构")

    base_world.set("name", "student_center")

    # Keep PX4 physics and Gazebo Sim system plugins.
    # Replace only scene entities, lights, models and GUI camera setup.
    remove_children(
        base_world,
        {
            "scene",
            "light",
            "model",
            "include",
            "gui",
        },
    )

    for tag in ("scene", "light", "model", "gui"):
        for element in legacy_world.findall(tag):
            base_world.append(copy.deepcopy(element))

    convert_script_materials(base_world)

    output_path.parent.mkdir(parents=True, exist_ok=True)

    try:
        ET.indent(base_tree, space="  ")
    except AttributeError:
        pass

    base_tree.write(
        output_path,
        encoding="utf-8",
        xml_declaration=True,
    )

    model_count = len(base_world.findall("model"))
    plugin_count = len(base_world.findall("plugin"))

    print(f"已生成：{output_path}")
    print(f"静态模型数量：{model_count}")
    print(f"Gazebo Sim 系统插件数量：{plugin_count}")


if __name__ == "__main__":
    main()
