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

    # Get the absolute path of this launch file
    this_file_path = os.path.realpath(__file__)
    this_dir = os.path.dirname(this_file_path)

    # Go up to workspace root (depending where your launch file is!)
    # Example: if your launch file is in src/plane_recognition_package/launch/, go up 3 levels:
    workspace_root = os.path.abspath(os.path.join(this_dir, '../../../../..'))

    # Now point to DATA/MIT_Recognition
    main_folder_path = os.path.join(workspace_root, 'DATA', 'MIT_One_Camera_Recognition')

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
    airplanes = ["airplane_1","airplane_2"]
    camera = ['1.0', '0.0']
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
    # Start the Node of camera recognition, and the movementment in the FOV:
    start_recog = Node(
        package='airplane_recognition',
        executable='airplane_recognition',
        output = 'screen',
        name = 'airplane_recognition',
        parameters=[{'use_sim_time': True}],
    )

    # Start to move the robor usinf the Waypoitns defined:
    start_robot_move = Node(
                package='airplane_recognition',
                executable='One_camera_recognition',
                output = 'screen',
                name = 'One_camera_recognition',
                parameters=[{
                    "airplane_name_own": airplanes[0],
                    "airplane_name_in": airplanes[1],
                    "output_folder": new_folder_path,
                    "fog_type": "exponential",        # Verify the type in the world file in the scene branch
                    "fog_density": 0.0115,             # The sanme goes for the fog density
                    "camera_resolution": "3776x2360", # Check the resultion in the urdf too
                    "Yolo_model": "Yolo_m", # Define the yolo model used.
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

    # Start the RViz in case you want to see the sensors values:
    rviz_config_dir = os.path.join(get_package_share_directory('plane_description'),'rviz','recognition.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        name='rviz_node',
        parameters=[{'use_sim_time': True}],
        arguments=['-d', rviz_config_dir])

    

    return launch.LaunchDescription([
        start_world,
        *launch_descriptions_airplanes,
        #start_recog,
        #wait_for_gazebo_node,
        #launch_start_robot_move,
        stop_simulation,
         RegisterEventHandler(
             OnProcessExit(
                 target_action=stop_simulation,
                 on_exit=[Shutdown()]
             )
         ),

    ])