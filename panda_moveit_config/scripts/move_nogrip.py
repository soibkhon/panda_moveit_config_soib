#!/usr/bin/env python3

from __future__ import print_function
import sys
import rospy
import moveit_commander
import geometry_msgs.msg
from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest, SwitchControllerResponse


class SimpleMove:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("simple_move", anonymous=True)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.move_group = moveit_commander.MoveGroupCommander("panda_arm")

    def move_x(self, distance):
        move_group = self.move_group

        pose_goal = move_group.get_current_pose().pose
        pose_goal.position.x += distance
        move_group.set_pose_target(pose_goal)

        success = move_group.go(wait=True)
        move_group.stop()
        move_group.clear_pose_targets()

        return success

def switch_controllers(start_controllers, stop_controllers):
    rospy.wait_for_service('/controller_manager/switch_controller')
    
    try:
        switch_controller = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)
        req = SwitchControllerRequest()
        req.start_controllers = start_controllers
        req.stop_controllers = stop_controllers
        req.strictness = SwitchControllerRequest.STRICT

        resp = switch_controller(req)
        if resp.ok:
            rospy.loginfo("Successfully switched controllers.")
        else:
            rospy.logerr("Failed to switch controllers.")
            
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")


def main():
    #rospy.init_node('controller_switcher')
    try:
        sm = SimpleMove()
        distance = 0.1  # 10 cm
        for _ in range(2):
            sm.move_x(distance)  # Move 10 cm forward
            sm.move_x(-distance)  # Move 10 cm backward
    except rospy.ROSInterruptException:
        pass
    
    stop_controllers = ['position_joint_trajectory_controller']
    
    # Start the Cartesian impedance controller
    start_controllers = ['tas_controllers/CartesianImpedanceController']

    switch_controllers(start_controllers, stop_controllers)
if __name__ == '__main__':
    main()
