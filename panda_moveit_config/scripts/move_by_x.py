#!/usr/bin/env python3

from __future__ import print_function
import sys
import rospy
import moveit_commander
import geometry_msgs.msg
import math

class SimpleMove:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("simple_move", anonymous=True)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.move_group = moveit_commander.MoveGroupCommander("panda_arm")
        self.move_group.set_max_velocity_scaling_factor(0.1)  # Adjust for smoother motion
        self.move_group.set_max_acceleration_scaling_factor(0.1)  # Adjust for smoother motion

    def circular_motion(self, radius, num_waypoints):
        move_group = self.move_group
        waypoints = []

        current_pose = move_group.get_current_pose().pose
        center_x = current_pose.position.x
        center_y = current_pose.position.y

        for i in range(num_waypoints + 1):
            theta = 2 * math.pi * i / num_waypoints
            x = center_x + radius * math.cos(theta)
            y = center_y + radius * math.sin(theta)

            pose_goal = geometry_msgs.msg.Pose()
            pose_goal.orientation = current_pose.orientation  # Keep the orientation constant
            pose_goal.position.x = x
            pose_goal.position.y = y
            pose_goal.position.z = current_pose.position.z
            waypoints.append(pose_goal)

        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints,   # waypoints to follow
            0.01,        # eef_step
            0.0)         # jump_threshold

        plan = move_group.retime_trajectory(self.robot.get_current_state(), plan, velocity_scaling_factor=0.5, acceleration_scaling_factor=0.5)
        move_group.execute(plan, wait=True)

def main():
    try:
        sm = SimpleMove()
        radius = 0.1  # Radius of the circle in meters
        num_waypoints = 300  # Number of waypoints to generate
        sm.circular_motion(radius, num_waypoints)  # Execute circular motion
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
