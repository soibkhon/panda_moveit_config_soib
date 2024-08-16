#!/usr/bin/env python3

import rospy
import sys
from moveit_commander import RobotCommander, PlanningSceneInterface, MoveGroupCommander
from geometry_msgs.msg import PoseStamped

def move_robot_x(distance):
    # Initialize MoveIt commander and rospy node
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_x_axis', anonymous=True)

    # Instantiate RobotCommander (interface to robot)
    robot = RobotCommander()
    
    # Instantiate PlanningSceneInterface (interface to world)
    scene = PlanningSceneInterface()
    
    # Instantiate MoveGroupCommander (interface to arm group)
    group = MoveGroupCommander("panda_arm")

    # Get the current pose of the end-effector
    current_pose = group.get_current_pose().pose

    # Modify the target pose
    target_pose = PoseStamped()
    target_pose.header.frame_id = "panda_link0"
    target_pose.pose.position.x = current_pose.position.x + distance
    target_pose.pose.position.y = current_pose.position.y
    target_pose.pose.position.z = current_pose.position.z
    target_pose.pose.orientation = current_pose.orientation

    # Set the target pose and move the robot
    group.set_pose_target(target_pose)
    plan = group.go(wait=True)

    # Ensure there is no residual movement
    group.stop()
    group.clear_pose_targets()

    # Shut down MoveIt commander
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        move_robot_x(0.05)
    except rospy.ROSInterruptException:
        pass
