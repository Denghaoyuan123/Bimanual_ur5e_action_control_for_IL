#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy, sys
import moveit_commander
from geometry_msgs.msg import PoseStamped
import time
import os
from RobotiqGripper import RobotiqHand


class MoveItFkDemo:
    def __init__(self):
        self.gripper = RobotiqHand()
        self.gripper.connect("192.168.0.121", 54321)
        self.gripper.reset()
        self.gripper.activate()
        self.gripper.wait_activate_complete()
        self.gripper_position = 0
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('moveit_fk_demo', anonymous=True)

        self.arm = moveit_commander.MoveGroupCommander('manipulator')
        
        self.arm.set_goal_position_tolerance(0.001)
        self.arm.set_goal_orientation_tolerance(0.001)
        
        self.arm.set_max_acceleration_scaling_factor(0.1)
        self.arm.set_max_velocity_scaling_factor(0.1)

        self.desired_execution_time = 5.0

        # Initialize pose
        init_goal = [0.02478, -2.2794, -1.595056, 3.884405, 0.023387, 0.013963]
        self.arm.set_joint_value_target(init_goal)
        self.arm.go()
        rospy.sleep(3)

        # Moving flag
        self.moving = False

        # Load the initial pose from file
        self.load_poses_from_file('poses.txt')

    def load_poses_from_file(self, filepath):
        if os.path.exists(filepath):
            try:
                with open(filepath, 'r') as file:
                    first_line = file.readline()
                    if first_line:
                        x, y, z, ox, oy, oz, ow, gs = map(float, first_line.strip().split())
                        pose = PoseStamped()
                        pose.header.frame_id = "base"
                        pose.pose.position.x = x
                        pose.pose.position.y = y
                        pose.pose.position.z = z
                        pose.pose.orientation.x = ox
                        pose.pose.orientation.y = oy
                        pose.pose.orientation.z = oz
                        pose.pose.orientation.w = ow
                        self.set_target_pose(pose)
                        if gs == 0:
                            self.gripper_position = 255
                        elif gs == 1:
                            self.gripper_position = 0
                        self.gripper.move(self.gripper_position, 0, 100)
            except IOError:
                rospy.logerr("Could not read file: %s", filepath)
        else:
            rospy.loginfo("No pose file found: %s, waiting for next timer", filepath)

    def set_target_pose(self, pose):
        self.arm.set_pose_target(pose)
        # Log the goal position and orientation
        rospy.loginfo(f"Setting goal position to: x={pose.pose.position.x}, y={pose.pose.position.y}, z={pose.pose.position.z}")
        rospy.loginfo(f"Setting goal orientation to: ox={pose.pose.orientation.x}, oy={pose.pose.orientation.y}, oz={pose.pose.orientation.z}, ow={pose.pose.orientation.w}")
        if not self.arm.go(wait=True):
            rospy.logerr("No valid plan found for the given pose.")
        self.arm.stop()
        self.arm.clear_pose_targets()

if __name__ == "__main__":
    try:
        demo = MoveItFkDemo()
        while not rospy.is_shutdown():
            time.sleep(0.5)  # Wait a bit before checking for new poses
            demo.load_poses_from_file('poses.txt')  # Attempt to reload poses from file
    except rospy.ROSInterruptException:
        pass
