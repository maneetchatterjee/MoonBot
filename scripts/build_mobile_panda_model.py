from __future__ import annotations

import copy
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
BASE_PATH = ROOT / "assets" / "mjcf" / "sawppy_base.xml"
PANDA_PATH = ROOT / "assets" / "mjcf" / "franka_emika_panda" / "panda.xml"
OUTPUT_PATH = ROOT / "assets" / "mjcf" / "moonbot_x.xml"


def required(root: ET.Element, query: str) -> ET.Element:
    element = root.find(query)
    if element is None:
        raise ValueError(f"Missing XML element: {query}")
    return element


def build() -> Path:
    base_tree = ET.parse(BASE_PATH)
    base_root = base_tree.getroot()
    panda_root = ET.parse(PANDA_PATH).getroot()

    base_default = required(base_root, "default")
    panda_default = required(panda_root, "default/default[@class='panda']")
    for pad_default in panda_default.findall(".//default"):
        if pad_default.get("class", "").startswith("fingertip_pad_collision"):
            pad_geom = required(pad_default, "geom")
            pad_geom.set("friction", "2.0 0.01 0.002")
            pad_geom.set("solref", "0.002 1")
            pad_geom.set("solimp", "0.99 0.9999 0.001")
            if pad_default.get("class") == "fingertip_pad_collision_1":
                pad_geom.set("size", "0.010 0.004 0.028")
                pad_geom.set("pos", "0 0.0055 0.0445")
    base_default.append(copy.deepcopy(panda_default))

    base_asset = required(base_root, "asset")
    for element in required(panda_root, "asset"):
        imported = copy.deepcopy(element)
        if imported.tag == "mesh" and "file" in imported.attrib:
            imported.set("file", f"franka_emika_panda/assets/{imported.get('file')}")
        base_asset.append(imported)

    panda_body = copy.deepcopy(required(panda_root, "worldbody/body[@name='link0']"))
    panda_body.set("pos", "0.14 0 0.143")
    panda_body.set("euler", "0 0.523599 0")
    hand = required(panda_body, ".//body[@name='hand']")
    ET.SubElement(hand, "site", name="end_effector", pos="0 0 0.1034", size="0.012")
    left_finger = required(panda_body, ".//body[@name='left_finger']")
    right_finger = required(panda_body, ".//body[@name='right_finger']")
    for side, finger in (("left", left_finger), ("right", right_finger)):
        for geom in finger.findall("geom"):
            collision_class = geom.get("class", "")
            if collision_class == "collision" or collision_class.endswith(("_2", "_3", "_4", "_5")):
                geom.set("contype", "0")
                geom.set("conaffinity", "0")
            elif collision_class == "fingertip_pad_collision_1":
                geom.set("name", f"{side}_jaw_liner_collision")
        ET.SubElement(
            finger,
            "geom",
            name=f"{side}_jaw_liner_visual",
            type="box",
            size="0.010 0.004 0.028",
            pos="0 0.0055 0.0445",
            rgba="0.08 0.42 0.48 1",
            contype="0",
            conaffinity="0",
            group="2",
        )
    ET.SubElement(
        left_finger,
        "site",
        name="left_fingertip",
        pos="0 0.0055 0.0445",
        size="0.010",
        rgba="1 0 0 0.25",
    )
    ET.SubElement(
        right_finger,
        "site",
        name="right_fingertip",
        pos="0 0.0055 0.0445",
        size="0.010",
        rgba="1 0 0 0.25",
    )
    required(base_root, "worldbody/body[@name='base']").append(panda_body)

    base_contact = required(base_root, "contact")
    for element in required(panda_root, "contact"):
        base_contact.append(copy.deepcopy(element))

    base_equality = required(base_root, "equality")
    for element in required(panda_root, "equality"):
        base_equality.append(copy.deepcopy(element))

    tendon = copy.deepcopy(required(panda_root, "tendon"))
    actuator_index = list(base_root).index(required(base_root, "actuator"))
    base_root.insert(actuator_index, tendon)

    actuator = required(base_root, "actuator")
    for index in range(1, 8):
        torque = 87 if index <= 4 else 12
        ET.SubElement(
            actuator,
            "motor",
            name=f"arm_motor_{index}",
            joint=f"joint{index}",
            ctrllimited="true",
            ctrlrange=f"-{torque} {torque}",
            forcelimited="true",
            forcerange=f"-{torque} {torque}",
        )
    ET.SubElement(
        actuator,
        "position",
        name="gripper_actuator",
        tendon="split",
        kp="2000",
        kv="50",
        ctrllimited="true",
        ctrlrange="0 0.04",
        forcelimited="true",
        forcerange="-70 70",
    )

    sensor = required(base_root, "sensor")
    ET.SubElement(
        sensor,
        "framepos",
        name="end_effector_position",
        objtype="site",
        objname="end_effector",
    )
    ET.SubElement(
        sensor,
        "framequat",
        name="end_effector_orientation",
        objtype="site",
        objname="end_effector",
    )
    ET.SubElement(
        sensor,
        "framelinvel",
        name="end_effector_velocity",
        objtype="site",
        objname="end_effector",
    )
    ET.SubElement(sensor, "touch", name="left_fingertip_contact", site="left_fingertip")
    ET.SubElement(sensor, "touch", name="right_fingertip_contact", site="right_fingertip")

    ET.indent(base_tree, space="  ")
    base_tree.write(OUTPUT_PATH, encoding="utf-8", xml_declaration=False)
    return OUTPUT_PATH


if __name__ == "__main__":
    print(build())
