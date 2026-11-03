import sys
from pathlib import Path
import unittest
from geometry_msgs.msg import PoseStamped
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/go2_bringup/scripts'))
from ground_goal_adapter import body_goal


class GroundGoalTest(unittest.TestCase):
    def test_surface_height_is_added_once_without_mutating_input(self):
        goal = PoseStamped()
        goal.header.frame_id = 'world'
        goal.pose.position.z = 3.0
        goal.pose.orientation.w = 2.0
        converted = body_goal(goal, .35, 'world')
        self.assertEqual(goal.pose.position.z, 3.0)
        self.assertAlmostEqual(converted.pose.position.z, 3.35)
        self.assertEqual(converted.pose.orientation.w, 1.0)

    def test_invalid_frame_and_quaternion_are_rejected(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.orientation.w = 0.0
        with self.assertRaises(ValueError):
            body_goal(goal, .35, 'world')
        goal.header.frame_id = 'world'
        with self.assertRaises(ValueError):
            body_goal(goal, .35, 'world')
        goal.pose.orientation.w = 1.0
        goal.pose.position.x = float('nan')
        with self.assertRaises(ValueError):
            body_goal(goal, .35, 'world')
