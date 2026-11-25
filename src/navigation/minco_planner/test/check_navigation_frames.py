"""Offline geometric regression examples; these do not execute the C++ plugins."""

import math
import unittest


def rotate(vector, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return c * vector[0] - s * vector[1], s * vector[0] + c * vector[1]


class NavigationFrameChecks(unittest.TestCase):
    def test_cold_start_velocity_uses_map_heading(self):
        # Base faces odom +x; odom +x points map +y. A moving robot must start
        # its map-frame trajectory along +y, not suddenly bend toward map +x.
        measured_body_velocity = (0.2, 0.0)
        planning_velocity = rotate(measured_body_velocity, math.pi / 2)
        self.assertAlmostEqual(planning_velocity[0], 0.0)
        self.assertAlmostEqual(planning_velocity[1], 0.2)
        wrong_odom_velocity = measured_body_velocity
        self.assertGreater(math.dist(planning_velocity, wrong_odom_velocity), 0.28)

    def test_reached_goal_with_translated_rotated_map(self):
        # odom robot (2,1) maps to map (9,0) under R90 and translation (10,-2).
        # The map goal (9.1,0.1) is only sqrt(.02) m away, despite dissimilar
        # coordinate numbers in the two frames.
        robot_odom = (2.0, 1.0)
        goal_map = (9.1, 0.1)
        goal_odom = rotate((goal_map[0] - 10, goal_map[1] + 2), -math.pi / 2)
        self.assertLess(math.dist(robot_odom, goal_odom), 0.3)
        self.assertGreater(math.dist(robot_odom, goal_map), 7.0)

    def test_equal_numbers_in_distinct_frames_are_not_arrival(self):
        robot_odom = (2.0, 0.0)
        goal_map = (2.0, 0.0)
        # map <- odom translation is +10 m on x.
        goal_odom = (goal_map[0] - 10, goal_map[1])
        self.assertEqual(math.dist(robot_odom, goal_map), 0.0)
        self.assertEqual(math.dist(robot_odom, goal_odom), 10.0)

    def test_heading_composition_matches_vector_frame_change(self):
        for map_odom_yaw in (-2.2, -0.8, 0.0, 0.7, math.pi / 2):
            for odom_base_yaw in (-1.2, 0.0, 2.6):
                body_velocity = (0.23, -0.11)
                expected = rotate(rotate(body_velocity, odom_base_yaw), map_odom_yaw)
                from_planning_pose = rotate(body_velocity, map_odom_yaw + odom_base_yaw)
                self.assertAlmostEqual(from_planning_pose[0], expected[0])
                self.assertAlmostEqual(from_planning_pose[1], expected[1])
                self.assertAlmostEqual(math.hypot(*from_planning_pose), math.hypot(*body_velocity))


if __name__ == "__main__":
    unittest.main()
