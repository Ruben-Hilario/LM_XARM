#!/usr/bin/env python3
"""
MoveIt Servo Keyboard Teleop — keyboard-only launch.

Prerequisites (run first in a separate terminal):
  ros2 launch xarm_moveit_servo xarm_moveit_servo_fake.launch.py dof:=7
  # or for xarm6:
  ros2 launch xarm_moveit_servo xarm_moveit_servo_fake.launch.py dof:=6

Then run this launch in a NEW terminal (it reads keys from stdin):
  ros2 launch xarm_control moveit_servo_keyboard.launch.py

Optional args:
  dof:=7            (default 7 — only affects move_group name label in logs)
  linear_speed:=0.1 (m/s,   default 0.10)
  angular_speed:=0.3 (rad/s, default 0.30)
  joint_speed:=0.5   (rad/s, default 0.50)

Key bindings:
  Arrow UP/DOWN    → X axis (forward / back)
  Arrow LEFT/RIGHT → Y axis (left / right)
  ; / .            → Z axis (up / down)
  [ / ]            → Yaw rotation
  W / E            → Base frame / End-effector frame
  1-7              → Joint jog (joint1..joint7)
  R                → Reverse joint direction
  Q                → Quit
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # ── tunable parameters ───────────────────────────────────────────────
        DeclareLaunchArgument('dof',           default_value='7',   description='DOF of the robot (for reference)'),
        DeclareLaunchArgument('linear_speed',  default_value='0.10', description='Cartesian linear speed (m/s)'),
        DeclareLaunchArgument('angular_speed', default_value='0.30', description='Cartesian angular speed (rad/s)'),
        DeclareLaunchArgument('joint_speed',   default_value='0.50', description='Joint jog speed (rad/s)'),
        DeclareLaunchArgument('base_frame',    default_value='link_base', description='Base / planning frame'),
        DeclareLaunchArgument('ee_frame',      default_value='link_eef',  description='End-effector frame'),

        # ── keyboard node (runs in this terminal, reads stdin) ───────────────
        Node(
            package='xarm_control',
            executable='moveit_servo_keyboard',
            name='moveit_servo_keyboard_node',
            output='screen',
            # stdin=True is implicit for Node; do NOT use prefix='xterm -e'
            # so keyboard input is read from the terminal that ran ros2 launch
            emulate_tty=True,
            parameters=[{
                'planning_frame': LaunchConfiguration('base_frame'),
                'base_frame':     LaunchConfiguration('base_frame'),
                'ee_frame':       LaunchConfiguration('ee_frame'),
                'linear_speed':   LaunchConfiguration('linear_speed'),
                'angular_speed':  LaunchConfiguration('angular_speed'),
                'joint_speed':    LaunchConfiguration('joint_speed'),
            }],
        ),
    ])
