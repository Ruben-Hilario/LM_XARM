git clone -b {ROS_DISTRO} https://github.com/xArm-Developer/xarm_ros2.git
cd xarm_ros2/
git submodule update --init --recursive
git pull --recurse-submodules
rosdep update
rosdep install --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
cd ../..
colcon build
source install/setup.bash

#If it fails:
# 
# sudo apt install -y \
# ros-${ROS_DISTRO}-control-msgs \
# ros-${ROS_DISTRO}-hardware-interface \
# ros-${ROS_DISTRO}-controller-manager-msgs \
# ros-${ROS_DISTRO}-ros-gz \
# ros-${ROS_DISTRO}-moveit \
# ros-${ROS_DISTRO}-moveit-servo \
# ros-${ROS_DISTRO}-moveit-visual-tools \
# ros-${ROS_DISTRO}-controller-manager \
# ros-${ROS_DISTRO}-joint-trajectory-controller \
# ros-${ROS_DISTRO}-joint-state-broadcaster
