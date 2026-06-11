from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    trigger_servo_command_type = ExecuteProcess(
        cmd=[
            'ros2', 'service', 'call', 
            '/servo_server/switch_command_type', 
            'moveit_msgs/srv/ServoCommandType', 
            '{command_type: 1}'
        ],
        output='screen'
    )

    free_control_node = Node(
        package='xarm_control',
        executable='free_control_node',
        name='free_control_node',
        output='screen',
    )

    return LaunchDescription([
        trigger_servo_command_type,
        free_control_node
    ])