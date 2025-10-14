from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    pkg_share = FindPackageShare('robot-localization')
    xacro_path = PathJoinSubstitution([pkg_share, 'urdf', 'rover.urdf.xacro'])

    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output="screen",
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('xacro_path')])}],
    )

    joint_state_pub_gui = Node(
        package = 'joint_state_publisher_gui',
        executable = 'joint_state_publisher_gui',
        name = 'joint_state_publisher_gui',
        output = 'screen'
    )

    rviz_node = Node(
        package = 'rviz2',
        executable = 'rviz2',
        name = 'rviz2',
        output = 'screen'
    )


    return LaunchDescription([
        DeclareLaunchArgument(
            'xacro_path',
            default_value = xacro_path,
            description = 'path to the xacro file'
        ), 
        DeclareLaunchArgument(
            'use_gui',
            default_value = 'True',
            description = 'Flag to enable joint_state_publisher_gui'
        ),
        robot_state_pub,
        joint_state_pub_gui,
        rviz_node
    ]
    )