
# Dual Arm Robotic System Repository
This repository is dedicated to receiving action from Imitation Learning and utilizing MoveIt! for planning and executing these actions on a dual Ur5e robotic system.

## Deployment Process

1. **Enter the dual_arm_ros directory:**
   ```bash
   cd demo_ws/
   ```
   - src/
      - Bimanual_Universal_Robots_ur5e_ROS_Driver/
        https://github.com/Denghaoyuan123/Bimanual_Universal_Robots_ur5e_ROS_Driver

      - realsense-ros/
        https://github.com/IntelRealSense/realsense-ros

      - Bimanual_ur5e_joystick_control
        https://github.com/Denghaoyuan123/Bimanual_ur5e_joystick_control

      - This repo
        https://github.com/Denghaoyuan123/Bimanual_ur5e_action_control_for_IL/tree/main

2. **Ensure the correct sourcing of the environment:**
   - **Right Arm Setup:**
     ```bash
     roslaunch ur_robot_driver ur5e_bringup_right.launch ns:=robot1 robot_ip:=xxx reverse_port:=50001 script_sender_port:=50002 trajectory_port:=50003 script_command_port:=50004
     roslaunch ur5e_moveit_config right_arm_launch.launch
     roslaunch control_robot demo1.launch task_name:=task1
     ```

   - **Left Arm Setup:**
     ```bash
     roslaunch ur_robot_driver ur5e_bringup_left.launch ns:=robot2 robot_ip:=xxx reverse_port:=50011 script_sender_port:=50012 trajectory_port:=50013 script_command_port:=50014
     roslaunch ur5e_moveit_config left_arm_launch.launch
     roslaunch control_robot demo2.launch task_name1:=task1
     ```

   - **Additional Commands:**
     ```bash
     rosrun ur5e_joystick_control get_obs.py
     roslaunch realsense2_camera rs_aligned_depth1.launch
