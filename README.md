# Build Docker

#Do ros setup
chmod +x ./setup.sh
./setup.sh

# Terminal 1
ros2 launch xarm_moveit_servo xarm_moveit_servo_fake.launch.py dof:=6 


#Terminal 2
ros2 launch xarm_control free_control_node
