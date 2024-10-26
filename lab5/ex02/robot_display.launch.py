from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['xacro', '$(find ex02_xacro_cleanup)/urdf/robot.urdf.xacro', '-o', '$(find ex02_xacro_cleanup)/urdf/robot.urdf'],
            output='screen'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            arguments=['$(find ex02_xacro_cleanup)/urdf/robot.urdf']
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', '$(find ex02_xacro_cleanup)/rviz/display.rviz']
        )
    ])
