import os
import re
from tracemalloc import stop
import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit


def generate_launch_description():

    # Define the frequency
    frequency = 10
    
    # Start the world and the spawn the airplanes in the world
    pkg_launch_bringup = "plane_bringup"

    # Folder Path:
    main_folder_path = "/home/adcl/AirplanePathFollower/DATA/Fligths"

    # Fligth FOlders inside:
    fligths = [f for f in os.listdir(main_folder_path) if os.path.isdir(os.path.join(main_folder_path,f))]
    # Extract the number of fligths:
    fligth_numbers = []
    for fligth in fligths:
        match = re.match(r'Flight(\d+)', fligth)
        if match:
            fligth_numbers.append(int(match.group(1)))

    # Determine the next fligth folder number
    if fligth_numbers:
        next_folder_number = max(fligth_numbers) + 1
    else:
        next_folder_number = 1

    # Generate the folder of the fligth:
    new_folder_name = f'Flight{next_folder_number}'
    new_folder_path = os.path.join(main_folder_path, new_folder_name)
    os.makedirs(new_folder_path)
    print(f"New folder created: {new_folder_path}")

    # Generate the folder of the state_data:
    state_folder_path = os.path.join(new_folder_path, "state_data")
    os.makedirs(state_folder_path)
    print(state_folder_path)

    # Generate the folder of performance metrics:
    performance_metrics_path = os.path.join(new_folder_path, "performance_metrics")
    os.makedirs(performance_metrics_path)
    print(performance_metrics_path)

    # Generate the detection data folder:
    detection_data_path = os.path.join(new_folder_path, "detection_data")
    os.makedirs(detection_data_path)
    print(detection_data_path)


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
    save_airplanes_info = []

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
        # Save the airplane information:
        save_airplane_info = Node(
            package="save_information",
            executable="save_airplane_information",
            name = f"{airplanes[i]}_save_info",
            output = "screen",
            parameters=[{
                "airplane_name": airplanes[i],
                "output_folder": state_folder_path,
                "frecuency": frequency,
            }]
        )
        save_airplanes_info.append(save_airplane_info)


    # Start the RViz in case you want to see the sensors values:
    rviz_config_dir = os.path.join(get_package_share_directory('plane_description'),'rviz','recognition.rviz')
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


    # Save the Performance mtrics in a csv file:
    save_performance_info = Node(
            package="save_information",
            executable="save_performance_metrics",
            name = "save_performance_info",
            output = "screen",
            parameters=[{
                "output_folder": performance_metrics_path,
                "frecuency": frequency,
            }]
        )
    
    # Save the Detection information in a csv file:
    save_detection_info = Node(
            package="save_information",
            executable="save_detection_information",
            name = "save_detection_information",
            output = "screen",
            parameters=[{
                "airplane_name_avoider": airplanes[0],
                "airplane_name_obstacle": airplanes[1],
                "output_folder": detection_data_path,
                "frecuency": frequency,
            }]
        )
    
    #  Shutdown Node:
    stop_simulation = Node(
        package='save_information',
        executable='stop_simulation',
        output = 'screen',
        name = 'stop_simulation'
    )

    # Video Recorder:
    save_video = Node(
        package='airplane_recognition',
        executable='video_generator',
        output = 'screen',
        name = 'video_generator'
    )





    return launch.LaunchDescription([
        start_world,
        *launch_descriptions_airplanes,
        camera_fusion,
        *save_airplanes_info,
        save_performance_info,
        #save_detection_info,
        rviz_node,
        #save_video,
        stop_simulation,
        RegisterEventHandler(
            OnProcessExit(
                target_action=stop_simulation,
                on_exit=[Shutdown()]
            )
        ),
    ])