from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # 

    # Find the launch file  in urdf_tutorial package and run.
    return LaunchDescription([
        IncludeLaunchDescription(PathJoinSubstitution([FindPackageShare("urdf_tutorial"), "launch", "display.launch.py"]),
        launch_arguments={
            'model': PathJoinSubstitution([FindPackageShare('urdf_testing'), 'urdf', '1_simple.urdf'])
        }.items()
        ),


        
        
        ])
