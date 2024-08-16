#!/usr/bin/env python3

import rospy
import sys
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray

class MoveItImpedanceControl:
    def __init__(self):
        # Initialize MoveIt Commander and ROS node
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('moveit_impedance_control', anonymous=True)

        # Instantiate a MoveGroupCommander for the panda arm
        self.group = moveit_commander.MoveGroupCommander("panda_arm")

        # Publisher to send pose goals to impedance controller
        self.impedance_pub = rospy.Publisher('/tas_controllers/CartesianImpedanceController/target_pose', PoseStamped, queue_size=10)

    def move_to_pose(self, target_pose):
        # Plan to the target pose
        self.group.set_pose_target(target_pose)
        plan = self.group.go(wait=True)
        self.group.stop()
        self.group.clear_pose_targets()

        # Send target pose to impedance controller
        if plan:
            self.impedance_pub.publish(target_pose)
            rospy.loginfo("Target pose sent to impedance controller")

def main():
    # Create MoveItImpedanceControl object
    controller = MoveItImpedanceControl()

    # Define target pose
    target_pose = PoseStamped()
    target_pose.header.frame_id = "panda_link0"
    target_pose.pose.position.x = 0.4
    target_pose.pose.position.y = 0.0
    target_pose.pose.position.z = 0.4
    target_pose.pose.orientation.x = 0.0
    target_pose.pose.orientation.y = 1.0
    target_pose.pose.orientation.z = 0.0
    target_pose.pose.orientation.w = 0.0

    # Move to the target pose and publish it to the impedance controller
    controller.move_to_pose(target_pose)

if __name__ == '__main__':
    main()
