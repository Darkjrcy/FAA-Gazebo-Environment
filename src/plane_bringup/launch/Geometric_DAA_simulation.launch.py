import os
import re
import launch
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.actions import RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


# Define the launcher and the nodes:
def generate_launch_description():

    # Define the UAVs characteristics:
    avoider_name = "airplane_1"
    avoider_waypoints = "10, 0, -10, 0; 20, 0, -15, 0"
    # Intruder 1:
    intruder_1_name = "airplane_2"
    intruder_1_waypoints = "10, 50, -10, 0; 20, 55, -5, 0"
    # Intruder 2:
    intruder_2_name = "airplane_3"
    intruder_2_waypoints = "10, 50, -10, 0; 20, 55, -5, 0"
    # Notice If you want to add more airplanes you would need to create mor json files on the Airplanes characteristics. 

    # Start the world and the spawn the airplanes in the world
    pkg_launch_bringup = "plane_bringup"

    start_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare(pkg_launch_bringup),
                'launch',
                'start_world.launch.py'
            ])
        ]),
        launch_arguments={
            'world': [os.path.join(get_package_share_directory('plane_bringup'), 'worlds', 'empty_world.world'), ''],
            'camera' : 'true'
        }.items()
    )

    # Spawn the airplanes:
    airplanes = [avoider_name,intruder_1_name,intruder_2_name]
    camera = ['2.0', '0.0', '0.0']
    launch_descriptions_airplanes = []
    launch_description_delete_airplanes = []

    for i in range(len(airplanes)):
        # Deletet the airplanes enitity
        delete_airplane = Node(package='save_information',
                executable='delete_entity',
                name = 'delete_entity',
                parameters=[{
                    "entity_name": airplanes[i],
                }]
            )
        launch_description_delete_airplanes.append(delete_airplane)

        # Spawn the airplane
        spawn_airplane = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare(pkg_launch_bringup),
                    'launch',
                    'airplane.launch.py'
                ])
            ]),
            launch_arguments={
                'mat_file': f'{airplanes[i]}.mat',
                'camera' : camera[i]
            }.items()
        )
        launch_descriptions_airplanes.append(spawn_airplane)
    
    # Node to wait for gazebo before starting the spawner:
    wait_for_gazebo_node = Node(
        package='save_information',
        executable='gazebo_waiter',
        name='gazebo_waiter',
        output='screen'
    )

    # DAA executable aluncher:
    daa_simulation = Node(
        package='plane_follow_exec',
        executable='daa_simulation',
        output = 'screen',
        name = 'daa_simulation',
        parameters=[{
            "avoider_name": avoider_name,
            "number_intruders": "1",                    # Note: Change with the number of intruders:
            "intruder_name_1": intruder_1_name,
            "avoider_waypoints": avoider_waypoints,
            "intruder_waypoints_1": intruder_1_waypoints, 
            # Note: For the intruders the initial number to start the name is with _0 with the aitplane nuber _2
        }]
    )

    # Register event handler so start_robot_move launches only after wait_for_gazebo_node exits
    launch_start_robot_move = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_for_gazebo_node,
            on_exit=[daa_simulation]
        )
    )

    #  Shutdown Node:
    stop_simulation = Node(
        package='save_information',
        executable='stop_simulation',
        output = 'screen',
        name = 'stop_simulation'
    )


    return launch.LaunchDescription([
        start_world,
        *launch_descriptions_airplanes,
        #save_video,
        wait_for_gazebo_node,
        #launch_start_robot_move,
        stop_simulation,
         RegisterEventHandler(
             OnProcessExit(
                 target_action=stop_simulation,
                 on_exit=[Shutdown()]
             )
         ),
    ])


