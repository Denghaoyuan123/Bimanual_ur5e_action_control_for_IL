#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy, sys
import moveit_commander
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray
import time
import os
import math
from RobotiqGripper import RobotiqHand


class MoveItFkDemo:
    def __init__(self):
        self.gripper = RobotiqHand()
        self.gripper.connect("192.168.0.144", 54321)
        self.gripper.reset()
        self.gripper.activate()
        self.gripper.wait_activate_complete()
        self.gripper_position = 200  # 初始化夹爪位置为关闭状态
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('moveit_fk_demo', anonymous=True)

        self.arm = moveit_commander.MoveGroupCommander('robot2_manipulator')
        
        self.arm.set_goal_position_tolerance(0.001)
        self.arm.set_goal_orientation_tolerance(0.001)
        
        self.arm.set_max_acceleration_scaling_factor(0.1)
        self.arm.set_max_velocity_scaling_factor(0.1)
        self.task_name = rospy.get_param('~task_name1', 'pick_two')

        self.desired_execution_time = 5.0
        self.previous_z = 0.54
        self.previous_y = -0.28
        self.gripper_publisher = rospy.Publisher('/gripper_state2', Float64MultiArray, queue_size=1)
        self.gripper_publisher.publish(Float64MultiArray(data=[self.gripper_position]))  # 发布初始夹爪状态

        # Moving flag
        self.moving = False
        # Load the initial pose from file
        self.init_pose_file('/home/pine/demo_ws/poses2.txt')
        
        # self.load_poses_from_file('poses.txt')
        self.initial_poses = {
            'handover': {
                'position': {'x': -0.13, 'y': -0.38, 'z': 0.385},
                'orientation': {'x': 0.0, 'y': -1.0, 'z': 0.0, 'w': 0.0}
            },
            'pick_two': {
                'position': {'x': -0.27, 'y': -0.38, 'z': 0.26},
                'orientation': {'x': 0.0, 'y': -1.0, 'z': 0.0, 'w': 0.0}
            },
            'pick_one': {  
                'position': {'x': -0.21, 'y': -0.38, 'z': 0.26},
                'orientation': {'x': 0.0, 'y': -1.0, 'z': 0.0, 'w': 0.0}
            },
            'press': {
                'position': {'x': -0.19, 'y': -0.41, 'z': 0.410},
                'orientation': {'x': 0.0, 'y': -1.0, 'z': 0.0, 'w': 0.0}
            },
            'lift': {
                'position': {'x': -0.15, 'y': -0.40, 'z': 0.43},
                'orientation': {'x': 0.0, 'y': -1.0, 'z': 0.0, 'w': 0.0}
            },
            'clothes': {
                'position': {'x': -0.19, 'y': -0.35, 'z': 0.33},
                'orientation': {'x': 0.0, 'y': -1.0, 'z': 0.0, 'w': 0.0}
            },
            'robot': {
                'position': {'x': -0.25, 'y': -0.37, 'z': 0.29},
                'orientation': {'x': 0.0, 'y': -1.0, 'z': 0.0, 'w': 0.0}
            },
            # 可以添加更多任务和对应的初始位姿
        }

        # 定义基于关节角度的初始位姿（以度为单位）
        self.initial_joint_angles_degrees = {
            'pingpang': [18.70, -80.62, 145.4, -60.96, 14.5, -180.99],
            'toothbrush': [18.70, -80.62, 145.4, -60.96, 14.5, -180.99],
            'pour': [18.70, -80.62, 145.4, -60.96, 14.5, -180.99],
            # 可以添加更多任务和对应的关节角度
        }

        self.set_initial_pose()

    def init_pose_file(self, filepath):
        # Initialize or reset the poses.txt file
        if os.path.exists(filepath):  # 如果文件存在
            with open(filepath, 'w') as file:
                file.write(",true")
                rospy.loginfo(f"File exists. Reset the state to true in: {filepath}")
        else:  # 如果文件不存在
            with open(filepath, 'w') as file:
                file.write(",true")
                rospy.loginfo(f"Created new pose file: {filepath} with initial state set to true.")
            
    def set_initial_pose(self):
        if self.task_name in self.initial_joint_angles_degrees:
            # 获取关节角度（以度为单位）
            joint_angles_degrees = self.initial_joint_angles_degrees[self.task_name]
            print(f"任务 '{self.task_name}' 使用关节角度作为初始位姿：{joint_angles_degrees}")

            # 将角度转换为弧度
            joint_angles_radians = [math.radians(angle) for angle in joint_angles_degrees]
            print(f"转换为弧度后的关节角度：{joint_angles_radians}")

            # 设置关节目标
            self.arm.set_joint_value_target(joint_angles_radians)

            # 规划并执行运动
            plan = self.arm.go(wait=True)
            if plan:
                print(f"任务 '{self.task_name}' 的关节角度目标已成功执行。")
            else:
                print(f"任务 '{self.task_name}' 的关节角度目标执行失败。")

            # 停止任何剩余的运动
            self.arm.stop()
            # 清除目标
            self.arm.clear_pose_targets()
        else:
            # 获取任务对应的初始位姿
            pose_data = self.initial_poses.get(self.task_name, self.initial_poses['pick_two'])
            position = pose_data['position']
            orientation = pose_data['orientation']
            print(f"任务 '{self.task_name}' 使用位置和方向作为初始位姿。")

            # 初始化位姿
            init_pose = PoseStamped()
            init_pose.header.frame_id = "robot2_base"
            init_pose.pose.position.x = position['x']
            init_pose.pose.position.y = position['y']
            init_pose.pose.position.z = position['z']
            init_pose.pose.orientation.x = orientation['x']
            init_pose.pose.orientation.y = orientation['y']
            init_pose.pose.orientation.z = orientation['z']
            init_pose.pose.orientation.w = orientation['w']

            # 设置位姿目标
            self.arm.set_pose_target(init_pose)

            # 规划并执行运动
            plan = self.arm.go(wait=True)
            if plan:
                print(f"任务 '{self.task_name}' 的位置和方向目标已成功执行。")
            else:
                print(f"任务 '{self.task_name}' 的位置和方向目标执行失败。")

            # 停止任何剩余的运动
            self.arm.stop()
            # 清除目标
            self.arm.clear_pose_targets()

        # 控制夹爪（根据您的实际需求进行调整）
        self.gripper.move(255, 180, 100) 
        self.gripper_position = 200  # 更新夹爪位置
        self.gripper_publisher.publish(Float64MultiArray(data=[self.gripper_position]))  # 发布初始夹爪状态

    def load_poses_from_file(self, filepath):
        if os.path.exists(filepath):
            try:
                with open(filepath, 'r+') as file:
                    lines = file.readlines()
                    if lines:
                        last_line = lines[-1].strip()
                        if ',' in last_line:
                            action, state = last_line.split(',')
                            if state.strip() == 'false':
                                # Perform action because the planner has set the flag to false
                                x, y, z, ox, oy, oz, ow, gs = map(float, action.strip().split())
                                # if not hasattr(self, 'previous_z'):
                                #     self.previous_z = z  # 将第一次的 z 设为 previous_z

                                delta_z = z - self.previous_z
                                delta_y = y - self.previous_y
                                rospy.loginfo(f"Delta Z: {delta_z}")

                                pose = PoseStamped()
                                pose.header.frame_id = "robot2_base"
                                pose.pose.position.x = x - 0.78
                                pose.pose.position.y = y
                                pose.pose.position.z = z
                                pose.pose.orientation.x = ox
                                pose.pose.orientation.y = oy
                                pose.pose.orientation.z = oz
                                pose.pose.orientation.w = ow

                                self.set_target_pose(pose)
                                self.previous_z = z
                                self.previous_y = y

                                if gs == 1.0:
                                    self.gripper.move(0, 180, 100)  # 打开夹爪
                                    self.gripper_position = 0  # 更新夹爪位置
                                    rospy.loginfo("Gripper opened.")
                                    self.gripper_publisher.publish(Float64MultiArray(data=[self.gripper_position]))  # 发布当前夹爪状态
                                elif gs == 0.0:
                                    self.gripper.move(255, 180, 100)  # 关闭夹爪
                                    self.gripper_position = 200  # 更新夹爪位置
                                    rospy.loginfo("Gripper closed.")
                                    self.gripper_publisher.publish(Float64MultiArray(data=[self.gripper_position]))  # 发布当前夹爪状态

                                rospy.sleep(0.5)  # 等待执行完毕
                                # After action, update the file to set state to true
                                file.seek(0)
                                file.write(f",true")
                                file.truncate()
                                rospy.loginfo("Action performed and file state updated to true.")
                            else:
                                rospy.loginfo("wait false.")
                        else:
                            rospy.logerr("File format incorrect, expecting action followed by state.")
            except IOError as e:
                rospy.logerr(f"Could not read file: {filepath} with error: {e}")
        else:
            rospy.loginfo("No pose file found: %s, initializing with default state.", filepath)
            # If no file, create one with the initial state set to true
            with open(filepath, 'w') as file:
                file.write(",true")
                
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
        rospy.loginfo("MoveItFkDemo started successfully.")
        rate = rospy.Rate(5)  # 设置循环频率为5Hz，约等于每0.2秒执行一次
        while not rospy.is_shutdown():
            # 持续发布当前夹爪状态
            demo.gripper_publisher.publish(Float64MultiArray(data=[demo.gripper_position]))
            # 加载新的位姿
            demo.load_poses_from_file('/home/pine/demo_ws/poses2.txt')  # Attempt to reload poses from file
            rate.sleep()  # 以设定的频率进行睡眠

    except rospy.ROSInterruptException:
        pass
