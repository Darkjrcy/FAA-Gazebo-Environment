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


def generate_launch_description():

    # Define the frequency
    frequency = 10

    # Folder Path:
    main_folder_path = "/home/adcl/AirplanePathFollower/DATA/MIT_Recognition"

    # Fligth FOlders inside:
    tests = [f for f in os.listdir(main_folder_path) if os.path.isdir(os.path.join(main_folder_path,f))]
    # Extract the number of tests:
    test_numbers = []
    for test in tests:
        match = re.match(r'Test_(\d+)', test)
        if match:
            test_numbers.append(int(match.group(1)))
    # Determine the next test folder number
    if test_numbers:
        next_folder_number = max(test_numbers) + 1
    else:
        next_folder_number = 1
    
    # Generate the folder of the test:
    new_folder_name = f'Test_{next_folder_number}'
    new_folder_path = os.path.join(main_folder_path, new_folder_name)
    os.makedirs(new_folder_path)
    print(f"New folder created: {new_folder_path}")

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
            'world': [os.path.join(get_package_share_directory('plane_bringup'), 'worlds', 'ERAU.world'), ''],
            'camera' : 'true'
        }.items()
    )

    # Spawn the airplanes:
    airplanes = ["airplane_1","airplane_2"]
    camera = ['2.0', '0.0']
    launch_descriptions_airplanes = []

    for i in range(len(airplanes)):
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


    # Start the RViz in case you want to see the sensors values:
    rviz_config_dir = os.path.join(get_package_share_directory('plane_description'),'rviz','Camera360.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        name='rviz_node',
        parameters=[{'use_sim_time': True}],
        arguments=['-d', rviz_config_dir])
    
    # Combine the images into the 360 degrees image:
    camera_fusion = Node(
        package='airplane_recognition',
        executable='camera_combination',
        output = 'screen',
        name = 'camera_combination',
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
        stop_simulation,
        RegisterEventHandler(
             OnProcessExit(
                 target_action=stop_simulation,
                 on_exit=[Shutdown()]
             )
         ),
    ])