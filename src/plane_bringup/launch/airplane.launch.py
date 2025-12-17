import os
import numpy as np
import json
import random

# Use ROS2 libraries:
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.actions import DeclareLaunchArgument
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import IncludeLaunchDescription
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.substitutions import Command
from launch.actions import SetLaunchConfiguration, LogInfo

# Generate the random cameras order:
def random_cam_inp(num: int, total: int=5):
    num = max(0, min(int(num), total))
    vec = [True]*num + [False]*(total - num)
    random.shuffle(vec)
    return vec


# Define a function to generate the robot info:
def launch_setup(context, *args,**kwargs):
    
    # obtain the matlab_file_name from the arguments:
    math_file_name = LaunchConfiguration('mat_file').perform(context)
    # Change it to be a json filename:
    json_file_name = math_file_name.replace('.mat', '.json')
    

    # Find the matlab file using the name:
    package_share_dir = get_package_share_directory('plane_bringup')
    json_file_path = os.path.join(package_share_dir, 'AirplaneCharacteristics', json_file_name)

    # Get the information from the json file:
    with open(json_file_path, 'r') as f:
        data = json.load(f)
    
   # Extract data:
    pose_1 = data['pose']
    rot_1 = data['rot']
    robot_name = data['robot_name']
    robot_imu = data['robot_imu']

    x_pos, y_pos, z_pos = TextSubstitution(text=str(pose_1[0])), TextSubstitution(text=str(pose_1[1])), TextSubstitution(text=str(pose_1[2]))
    R_rot, P_rot, Y_rot = TextSubstitution(text=str(rot_1[0])), TextSubstitution(text=str(rot_1[1])), TextSubstitution(text=str(rot_1[2]))

    # Check if the Airplane needs to have vision sensoring or not:
    camera_value = LaunchConfiguration('camera').perform(context)
    number_cameras = int(LaunchConfiguration("num_cameras").perform(context))
    # Check for which type of camera you want to use:
    camera_exists = camera_value in ["1.0"]
    camera_360_exists = camera_value in ["2.0"]
    varaible_camera_exitst = camera_value in ["3.0"]
    package_description = "plane_description"
    
    if camera_exists:
        xacro_file = 'Airplane_with_Camera/main_plane.xacro' 
    elif camera_360_exists:
        xacro_file = 'Airplane_with_360Camera/main_plane.xacro'
    elif varaible_camera_exitst:
        xacro_file = 'Airplane_variable_Camera/main_plane.xacro'
    else:
        xacro_file = 'Airplane/main_plane.xacro'
    
    # Define the urdf file path:
    robot_desc_path = os.path.join(get_package_share_directory(package_description), "urdf", xacro_file)

    # Publish the robot:
    if varaible_camera_exitst:
        cam_inputs = random_cam_inp(number_cameras)
        cam_str = ['true' if b else 'false' for b in cam_inputs]
        robot_state_publisher_node = Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                namespace=robot_name,
            parameters=[{'frame_prefix': robot_name+'/', 
                         'use_sim_time': True, 
                         'robot_description': ParameterValue(
                             Command(['xacro ', robot_desc_path, 
                                                       ' robot_name:=', robot_name, 
                                                       ' robot_imu:=', robot_imu,
                                                       ' use_camera1:=', TextSubstitution(text=cam_str[0]),
                                                       ' use_camera2:=', TextSubstitution(text=cam_str[1]),
                                                       ' use_camera3:=', TextSubstitution(text=cam_str[2]),
                                                       ' use_camera4:=', TextSubstitution(text=cam_str[3]),
                                                       ' use_camera5:=', TextSubstitution(text=cam_str[4]),]),
                                                       value_type=str)
            }],
            output="screen"
        )
    elif camera_360_exists:
        cam_str = ['true','true','true','true','true']
        robot_state_publisher_node = Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                namespace=robot_name,
            parameters=[{'frame_prefix': robot_name+'/', 
                         'use_sim_time': True, 
                         'robot_description': Command(['xacro ', robot_desc_path, 
                                                       ' robot_name:=', robot_name, 
                                                       ' robot_imu:=', robot_imu])}],
            output="screen"
        )
    else:
        cam_str = ['false','false','false','false','false']
        robot_state_publisher_node = Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                namespace=robot_name,
            parameters=[{'frame_prefix': robot_name+'/', 
                         'use_sim_time': True, 
                         'robot_description': Command(['xacro ', robot_desc_path, 
                                                       ' robot_name:=', robot_name, 
                                                       ' robot_imu:=', robot_imu])}],
            output="screen"
        )


    # Send the camera string to the bigger aluncher:
    actions = [
        SetLaunchConfiguration(f'{robot_name}_cam1', TextSubstitution(text=cam_str[0])),
        SetLaunchConfiguration(f'{robot_name}_cam2', TextSubstitution(text=cam_str[1])),
        SetLaunchConfiguration(f'{robot_name}_cam3', TextSubstitution(text=cam_str[2])),
        SetLaunchConfiguration(f'{robot_name}_cam4', TextSubstitution(text=cam_str[3])),
        SetLaunchConfiguration(f'{robot_name}_cam5', TextSubstitution(text=cam_str[4])),
        LogInfo(msg=f"{robot_name}_cam: [compute_cams] cam flags: {cam_str}")
    ]

    # Spawn the robot in Gazebo:
    start_gazebo_ros_spawner_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        namespace=robot_name,
        output='screen',
        arguments=['-entity',
                   robot_name,
                   '-x', x_pos, '-y', y_pos,'-z', z_pos,'-R',R_rot,'-P',P_rot,'-Y',Y_rot,
                   '-topic', 'robot_description',
                   '-timeout', '120.0'
                   ]
    )

    return [*actions, robot_state_publisher_node, start_gazebo_ros_spawner_cmd]





def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
   