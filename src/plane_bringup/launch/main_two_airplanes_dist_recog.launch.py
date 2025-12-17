import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    
    # Start the world and the spawn the airplanes in the world
    pkg_launch_bringup = "plane_bringup"

    start_the_two_airplanes_model= IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare(pkg_launch_bringup),
                'launch',
                'main_two_airplanes.launch.py'
            ])
        ])
    )

    # Start the Node of camera recognition, and the movementment in the FOV:
    start_recog = Node(
        package='airplane_recognition',
        executable='airplane_recognition',
        output = 'screen',
        name = 'airplane_recognition',
        parameters=[{'use_sim_time': True}],

    )

    
    # Move the RObot to do a treshold distribution of detection
    start_robot_move = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='airplane_recognition',
                executable='airplane_recog_fov',
                output = 'screen',
                name = 'airplane_recog_fov',
                parameters=[{'use_sim_time': True}]
            )
        ]
    )



    return LaunchDescription([
        start_the_two_airplanes_model,
        start_recog,
        start_robot_move,
    ])