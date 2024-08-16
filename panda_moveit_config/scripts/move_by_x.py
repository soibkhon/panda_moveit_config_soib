#!/usr/bin/env python3

from __future__ import print_function
import sys
import rospy
import moveit_commander
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output as outputMsg

class SimpleMove:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("simple_move", anonymous=True)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.move_group = moveit_commander.MoveGroupCommander("panda_arm")
        self.gripper_pub = rospy.Publisher("Robotiq2FGripperRobotOutput", outputMsg.Robotiq2FGripper_robot_output, queue_size=10)
        rospy.sleep(2)  # Allow some time for initialization

    def move_joints(self, joint_goals):
        move_group = self.move_group

        move_group.set_joint_value_target(joint_goals)
        success = move_group.go(wait=True)
        move_group.stop()
        move_group.clear_pose_targets()

        return success
    
    def create_command(self, open_gripper):
        """Create a command to open or close the gripper."""
        command = outputMsg.Robotiq2FGripper_robot_output()
        command.rACT = 1
        command.rGTO = 1
        command.rSP = 255
        command.rFR = 150
        command.rPR = 0 if open_gripper else 255
        return command

    def control_gripper(self, open_gripper):
        """Control the gripper to open or close."""
        command = self.create_command(open_gripper)
        self.gripper_pub.publish(command)
        rospy.sleep(1)  # Allow time for the gripper to move

def main():
    try:
        sm = SimpleMove()

        # Define the target joint values (in radians)
        joint_goals = [-0.632, -0.127, 0.425, -2.691, 0.105, 2.577, 0.497]

        # Move joints to the specified angles
        sm.move_joints(joint_goals)

        # Close the gripper
        sm.control_gripper(False)

        # Move joints back to another set of specified angles
        joint_goals = [1.568, 0.139, -1.497, -1.675, 0.754, 2.218, 0.608]
        sm.move_joints(joint_goals)

        # Open the gripper
        sm.control_gripper(True)

    except rospy.ROSInterruptException:
        pass

def set_cartesian_impedance(self, stiffness_values, damping_values):
    set_cartesian_impedance_service = rospy.ServiceProxy('/panda_arm/cartesian_impedance_controller/set_cartesian_impedance', SetCartesianImpedance)
    request = SetCartesianImpedanceRequest()
    request.stiffness.linear.x, request.stiffness.linear.y, request.stiffness.linear.z = stiffness_values[:3]
    request.stiffness.angular.x, request.stiffness.angular.y, request.stiffness.angular.z = stiffness_values[3:]
    request.damping.linear.x, request.damping.linear.y, request.damping.linear.z = damping_values[:3]
    request.damping.angular.x, request.damping.angular.y, request.damping.angular.z = damping_values[3:]
    set_cartesian_impedance_service(request)

if __name__ == '__main__':
    main()
