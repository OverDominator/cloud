import re
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


class SimpleSceneFootprintTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.package = Path(__file__).resolve().parents[1]
        cls.project = cls.package.parents[1] / 'unity'

    def test_navigation_layers_share_scale_one_footprint(self):
        system = ET.parse(
            self.package / "launch/system_unity_fire.launch"
        ).getroot()
        args = {
            node.attrib["name"]: float(node.attrib["default"])
            for node in system.findall("arg")
            if node.attrib.get("name") in {
                "vehicleLength", "vehicleWidth", "safetyMargin"
            }
        }
        self.assertEqual(args, {
            "vehicleLength": 1.35,
            "vehicleWidth": 1.80,
            "safetyMargin": 0.30,
        })

        far = ET.parse(
            self.package / "launch/far_unity_fire.launch"
        ).getroot()
        robot_dim = next(
            float(node.attrib["value"])
            for node in far.findall("param")
            if node.attrib.get("name") == "/far_planner/robot_dim"
        )
        self.assertEqual(robot_dim, 2.0)

    def test_far_adapter_loads_private_recovery_configuration(self):
        system = ET.parse(
            self.package / "launch/system_unity_fire.launch"
        ).getroot()
        controller = next(
            node for node in system.findall("node")
            if node.attrib.get("name") == "local_wall_recovery_controller"
        )
        loads = [
            entry for entry in controller.findall("rosparam")
            if entry.attrib.get("command") == "load"
        ]
        self.assertEqual(len(loads), 1)
        self.assertIn("far_command_adapter.yaml", loads[0].attrib.get("file", ""))

    def test_planner_encloses_but_does_not_triple_unity_collider(self):
        scene = (
            self.project / "Assets/Scenes/FireRescue_Simplified.unity"
        ).read_text(encoding="utf-8")
        robot_start = scene.index("  m_Name: TrackedRobot")
        collider = scene[robot_start:robot_start + 5000]
        match = re.search(
            r"m_Size: \{x: ([0-9.]+), y: ([0-9.]+), z: ([0-9.]+)\}",
            collider,
        )
        self.assertIsNotNone(match)
        unity_width, _, unity_length = map(float, match.groups())
        self.assertGreaterEqual(1.80, unity_width)
        self.assertGreaterEqual(1.35, unity_length)
        self.assertLess(1.80, unity_width * 1.5)
        self.assertLess(1.35, unity_length * 1.5)


if __name__ == "__main__":
    unittest.main()
