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
    avoider_waypoints = (
        "4000, 0, -280, 80; "
        "5200, 0, -300, 85; "
        "6400, 0, -310, 90; "
        "7600, 0, -320, 90; "
        "8800, 0, -325, 90; "
        "9200, 0, -325, 90; "
        "10000, 0, -325, 90; "
        "12000, 0, -325, 90; "
        "14000, 0, -325, 90; "
        "16000, 0, -340, 90"
        " % "
        "4000, 0, -280, 80; "
        "5200, 0, -300, 85; "
        "6400, 0, -310, 90; "
        "7600, 0, -320, 90; "
        "8800, 0, -325, 90; "
        "9200, 0, -325, 90; "
        "10000, 0, -325, 90; "
        "12000, 0, -325, 90; "
        "14000, 0, -325, 90; "
        "16000, 0, -340, 90"
        " % "
        "4000, 0, -280, 80; "
        "5200, 0, -300, 85; "
        "6400, 0, -310, 90; "
        "7600, 0, -320, 90; "
        "8800, 0, -325, 90; "
        "9200, 0, -325, 90; "
        "10000, 0, -325, 90; "
        "12000, 0, -325, 90; "
        "14000, 0, -325, 90; "
        "16000, 0, -340, 90"
        " % "
        "4000, 0, -280, 80; "
        "5200, 0, -280, 80; "
        "6400, 0, -280, 80; "
        "7600, 0, -280, 80; "
        "8800, 0, -280, 80; "
        "10000, 0, -280, 80"
    )

    # Intruder 1
    intruder_1_name = "airplane_2"
    intruder_1_waypoints = (
        "7800, 0, -300, 80; "
        "7600, 0, -300, 80; "
        "6400, 0, -300, 80; "
        "5200, 0, -300, 80; "
        "4000, 0, -300, 80; "
        "1000, 0, -300, 80; "
        "-800, 0, -300, 80"
        " % "
        "6150, 1230, -300, 50; "
        "7739, 1230, -300, 50; "
        "9476, 1230, -300, 50; "
        "11178, 1230, -300, 50; "
        "12616, 1230, -300, 50; "
        "14000, 1230, -300, 50"
        " % "
        "6500, 1800, -300, 80; "
        "6500, -1284, -300, 80; "
        "6500, -3443, -300, 80; "
        "6500, -6000, -300, 80; "
        "6500, -8608, -300, 80; "
        "6500, -11000, -300, 80"
        " % "
        "9800, 200, -300, 80; "
        "9600, 200, -300, 80; "
        "8400, 200, -300, 80; "
        "7200, 200, -300, 80; "
        "6000, 200, -300, 80; "
        "3800, 200, -300, 80"
    )

    # Intruder 2
    intruder_2_name = "airplane_3"
    intruder_2_waypoints = (
        "9000, 4800, -325, 80; "
        "9000, -1284, -325, 80; "
        "9000, -3443, -325, 80; "
        "9000, -6000, -325, 80; "
        "9000, -8608, -325, 80; "
        "9000, -11000, -325, 80"
        " % "
        "10000, -4800, -300, 80; "
        "9867, -3278, -290, 85; "
        "9679, -834, -280, 85; "
        "9415, 2595, -250, 90; "
        "9241, 4855, -240, 90; "
        "9000, 8000, -230, 90"
        " % "
        "10500, -6000, -300, 80; "
        "10367, -4278, -290, 85; "
        "10179, -1834, -280, 85; "
        "9915, 1595, -250, 90; "
        "9741, 3855, -240, 90; "
        "9500, 7000, -230, 90"
        " % "
        "9800, -200, -300, 80; "
        "9600, -200, -290, 85; "
        "8400, -200, -280, 85; "
        "7200, -200, -250, 90; "
        "6000, -200, -240, 90; "
        "3800, -200, -230, 90"
    )


    # Notice If you want to add more airplanes you would need to create mor json files on the Airplanes characteristics. 


    # World file:
    world = 'empty_world.world'




    # Main folder path where tehe flights are saved:
    main_folder_path = "/home/adcl/FAA-Gazebo-Environment/DATA/DAA_Flights" # Notice Change the folder with the path to your fodler DATA to save it !
    # Create tehe flight folder inside:
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

    # ChANge in case you want to use GPU in the YOLO detection model.
    use_gpu = True 
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
            'world': [os.path.join(get_package_share_directory('plane_bringup'), 'worlds', world), ''],
            'camera' : 'true'
        }.items()
    )
    

    # Spawn the airplanes:
    airplanes = [avoider_name,intruder_1_name,intruder_2_name]   # ADD THE INTRUDER TO THW SPAWNING SYSTEM TO.    
    camera = ['2.0', '0.0', '0.0']
    camera_numbers = ['5', '0', '0']
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
                'camera' : camera[i],
                'num_cameras' : camera_numbers[i],
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
            "data_directory": new_folder_path,
            "save_adsb_info": True,                     # Save the informatio that the ADS-B sends from the intruders
            "number_intruders": "2",                    # Note: Change with the number of intruders:
            "intruder_name_0": intruder_1_name,
            "intruder_name_1": intruder_2_name,
            "avoider_waypoints": avoider_waypoints,
            "intruder_waypoints_0": intruder_1_waypoints, 
            "intruder_waypoints_1": intruder_2_waypoints, 
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

    # Luanch the adsb for the recivever:
    adsb_intruder_1 = Node(
        package='airplane_gazebo_plugin',      
        executable='adsb_sensor',             
        name='adsb_sensor_launcher',
        output='screen',
        emulate_tty=True,
        arguments=["airplane_2", "imu_2", '--ros-args'],
    )
    adsb_intruder_2 = Node(
        package='airplane_gazebo_plugin',      
        executable='adsb_sensor',             
        name='adsb_sensor_launcher',
        output='screen',
        emulate_tty=True,
        arguments=["airplane_3", "imu_3", '--ros-args'],
    )

    # Combine the images into the 360 degrees image:
    camera_fusion = Node(
        package='airplane_recognition',
        executable='camera_combination',
        output = 'screen',
        name = 'camera_combination',
        parameters=[{
            "use_gpu": use_gpu,
        }]
    )


    return launch.LaunchDescription([
        start_world,
        *launch_descriptions_airplanes,
        #save_video,
        adsb_intruder_1,
        adsb_intruder_2,
        camera_fusion,
        wait_for_gazebo_node,
        launch_start_robot_move,
        stop_simulation,
         RegisterEventHandler(
             OnProcessExit(
                 target_action=stop_simulation,
                 on_exit=[Shutdown()]
             )
         ),
    ])


