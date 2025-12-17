import os
import re
import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node

def generate_launch_description():

    # Folder Path:
    main_folder_path = "/home/adcl/AirplanePathFollower/DATA/MIT_Recognition"

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
            'world': [os.path.join(get_package_share_directory('plane_bringup'), 'worlds', 'ERAU_ideal.world'), ''],
            'camera' : 'true'
        }.items()
    )

    # Spawn the airplanes:
    airplanes = ["airplane_1","airplane_2"]
    camera = ['3.0', '0.0']
    camera_numbers = ['3', '0']
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
                'camera' : camera[i],
                'num_cameras' : camera_numbers[i],
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
                "output_folder": state_folder_path
            }]
        )
        save_airplanes_info.append(save_airplane_info)
    
    # Save the ropic information:
    Airplane_1_save = launch_ros.actions.Node(
        package='rosbag2_recorder',
        executable='record',
        arguments=['-o', 'airplane_1_bag', '/real_time', '/sim_time', '/airplane_1/odom','/airplane_1/score','/RAM_usage','/CPU_usage'],
        output = 'screen'
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



    return LaunchDescription([
        start_world,
        *launch_descriptions_airplanes, # Unpacks the launch files
        #Airplane_1_save,
        #rviz_node,
    ])