# Build Docker

#Do ros setup
chmod +x ./setup.sh
./setup.sh

# Terminal 1
ros2 launch xarm_moveit_servo xarm_moveit_servo_fake.launch.py dof:=7 add_gripper:=true


#Terminal 2
ros2 run xarm_control free_control_node

Terminal 3 
ros2 service call /servo_server/switch_command_type moveit_msgs/srv/ServoCommandType "{command_type: 1}"

#Optional before moving robot
ros2 run xarm_moveit_servo xarm_keyboard_input 
