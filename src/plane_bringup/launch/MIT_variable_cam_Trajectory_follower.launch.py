import json
import os
from pathlib import Path
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
from launch.substitutions import LaunchConfiguration
from launch.actions import LogInfo
from launch_ros.parameter_descriptions import ParameterValue



def generate_launch_description():

    # Folder Path to save the information:
    main_folder_path = "/home/adcl/AirplanePathFollower/DATA/MIT_Fligths" # Notice Change the folder with the path to your fodler DATA to save it !

    # Launch follower varaibles:
    ownship_name = "airplane_1"
    intruder_name = "airplane_2"

    # Define the encoutner list, you can do it manually, but for the MIT simualtions they can be choose from hte detection_summary.
    package_share_directory_launch = Path(get_package_share_directory('plane_bringup'))
    json_ecounter_list_path = package_share_directory_launch / "Detection_encounters" / "encounters_tcpa.json"
    # Define the number of encounter you can to simulate:
    strating_point = 920
    number_encounters = 1000
    # Open the json file and add the encoutners id in a list:
    with json_ecounter_list_path.open("r", encoding="utf-8") as fil:
        encounters_summary = json.load(fil)
    # Save the information in a list:
    entries = []
    if isinstance(encounters_summary, list):
        for item in encounters_summary:
            try:
                _id = int(item["id"])
                _tcpa = int(item["tcpa"])
                entries.append((_id, _tcpa))
            except (KeyError, TypeError, ValueError):
                # skip malformed rows
                continue
    elif isinstance(encounters_summary, dict):
        for k, v in encounters_summary.items():
            try:
                _id = int(k)
                _tcpa = int(v)
                entries.append((_id, _tcpa))
            except (TypeError, ValueError):
                continue
    # Take only the number of enocunters you want to use:
    entries = entries[strating_point:number_encounters]
    # Save both as a string with the encounters set and the times set:
    encounter_list = ",".join(str(_id) for _id, _ in entries)
    tcpa_list = ",".join(str(_tcpa) for _, _tcpa in entries)
    # ChANge in case you want to use GPU in the YOLO detection model.
    use_gpu = True 


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
            'world': [os.path.join(get_package_share_directory('plane_bringup'), 'worlds', 'empty_world.world'), ''],
            'camera' : 'true'
        }.items()
    )

    # Spawn the airplanes:
    airplanes = [ownship_name,intruder_name]
    camera = ['3.0', '0.0']
    camera_numbers = ['3', '0']
    launch_descriptions_airplanes = []
    launch_description_delete_airplanes = []
    cameras_config = {}

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
                'camera' : camera[i],
                'num_cameras' : camera_numbers[i],
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
        executable='variable_camera_combination',
        output = 'screen',
        name = 'variable_camera_combination',
        parameters=[{
            "use_gpu": use_gpu,
        }]
    )

    # Node to wait for gazebo before starting the spawner:
    wait_for_gazebo_node = Node(
        package='save_information',
        executable='gazebo_waiter',
        name='gazebo_waiter',
        output='screen'
    )

    # Strat teh robot simulation of the trajectory movement:
    start_robot_move = Node(
        package='plane_follow_exec',
        executable='mit_encounter_follower',
        output = 'screen',
        name = 'mit_encounter_follower',
        parameters=[{
            "waypoint_list": encounter_list,
            "own_airplane": ownship_name,
            "in_airplane": intruder_name,
            "data_directory": new_folder_path,
            'tcpa_list': tcpa_list,
            "frecuency": 10,
            "fog_type": "exponential",        # Verify the type in the world file in the scene branch
            "fog_density": 0.01,             # The sanme goes for the fog density
            "camera_noise_type": "Gaussian",  # Check the noise type of the cameras in the Airplane gazebo section urdf
            "camera_noise_std": 0.00,         # Do the same for the standard deviatio
            "camera_resolution": "3776x2360", # Check the resultion in the urdf too
            "clutter": "Empty_world", # Check the world file you are inputing
            "Yolo_model": "Yolo_m", # Define the yolo model used.
            "Frontal_camera_active": ParameterValue(
            LaunchConfiguration(f'{ownship_name}_cam1'), value_type=str),
            "Frontal_rigth_camera_active": ParameterValue(
                LaunchConfiguration(f'{ownship_name}_cam2'), value_type=str),
            "Back_rigth_camera_active": ParameterValue(
                LaunchConfiguration(f'{ownship_name}_cam3'), value_type=str),
            "Back_left_camera_active": ParameterValue(
                LaunchConfiguration(f'{ownship_name}_cam4'), value_type=str),
            "Frontal_left_camera_active": ParameterValue(
                LaunchConfiguration(f'{ownship_name}_cam5'), value_type=str),
        }]
    )

    # Register event handler so start_robot_move launches only after wait_for_gazebo_node exits
    launch_start_robot_move = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_for_gazebo_node,
            on_exit=[start_robot_move]
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
        camera_fusion,
        #save_video,
        wait_for_gazebo_node,
        launch_start_robot_move,
        #rviz_node,
        stop_simulation,
         RegisterEventHandler(
             OnProcessExit(
                 target_action=stop_simulation,
                 on_exit=[Shutdown()]
             )
         ),
    ])