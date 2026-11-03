import importlib.util
from pathlib import Path
import tempfile
import time
import unittest

import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from ugv_nav4d_ros2.msg import MissionStatus, OperationalZoneArray, RouteRisk


module_path = Path(__file__).resolve().parents[1] / 'src/ugv_nav4d_ros2/scripts/field_operations.py'
spec = importlib.util.spec_from_file_location('field_operations', module_path)
field_operations = importlib.util.module_from_spec(spec)
spec.loader.exec_module(field_operations)


class FieldOperationsStateTest(unittest.TestCase):
    def test_late_start_receives_existing_mission(self):
        rclpy.init()
        publisher_node = rclpy.create_node('mission_state_fixture')
        health_node = None
        try:
            qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            messages = [
                ('execution_status', MissionStatus(state=MissionStatus.PAUSED,
                                                  can_resume=True)),
                ('planner_status', MissionStatus(state=MissionStatus.AWAITING_APPROVAL)),
                ('route_risk', RouteRisk(valid=True)),
                ('operational_zones', OperationalZoneArray()),
            ]
            publishers = []
            for topic, message in messages:
                publisher = publisher_node.create_publisher(
                    type(message), '/ugv_nav4d_ros2/' + topic, qos)
                publisher.publish(message)
                publishers.append(publisher)
            with tempfile.TemporaryDirectory() as directory:
                health_node = field_operations.FieldOperations()
                health_node.set_parameters([Parameter(
                    'mission_audit_file', value=str(Path(directory) / 'audit.jsonl'))])
                deadline = time.monotonic() + 8.0
                while time.monotonic() < deadline:
                    rclpy.spin_once(health_node, timeout_sec=0.1)
                    if all((health_node.last_execution, health_node.last_planner_status,
                            health_node.last_route_risk)):
                        break
                self.assertIsNotNone(health_node.last_execution)
                self.assertTrue(health_node.last_execution.can_resume)
                self.assertEqual(health_node.last_execution.state, MissionStatus.PAUSED)
                self.assertIsNotNone(health_node.last_planner_status)
                self.assertEqual(health_node.last_planner_status.state,
                                 MissionStatus.AWAITING_APPROVAL)
                self.assertIsNotNone(health_node.last_route_risk)
                self.assertTrue(health_node.last_route_risk.valid)
        finally:
            if health_node is not None:
                health_node.destroy_node()
            publisher_node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
