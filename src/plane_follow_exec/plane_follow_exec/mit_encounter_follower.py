# Python libraries:
from encodings.punycode import T
from mimetypes import init
import os
import math
from pickle import TRUE
from tkinter import PROJECTING, S
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import time
import random

# Library to change the orientation to quternion:
from scipy.spatial.transform import Rotation as R

# Library to generate specific QoS 
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# ROS2 Packages:
import rclpy
from rclpy.node import Node
# Messages and services:
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger
from airplane_gazebo_plugin.msg import AirplaneKineticModelInfo, AdsbData
from custom_msgs.msg import Yolov11Inference360
# Service to spawn the airplane in a position 
from gazebo_msgs.srv import SetEntityState

# Library to start process on the command window:
import subprocess


# Start the executable:
class MITEncounterFollower(Node):
    def __init__(self):
        # Generate the node:
        super().__init__('mit_encounter_follower')

        # Declare the argunments for the simualtion to work:
        self.declare_parameter('waypoint_list', '779') # List of the encoutners in the MIT dataset
        self.declare_parameter('tcpa_list', '121') # List of the time of CPA for each encoutner set
        self.declare_parameter('own_airplane', 'airplane_1') # Ownship model
        self.declare_parameter('in_airplane', 'airplane_2') # Intruder model
        self.declare_parameter('data_directory', '/home/adcl/AirplanePathFollower/DATA/MIT_Fligths') # Folder to save the inforamtion
        self.declare_parameter('frecuency', 10) # Frequency to save the information in Hz

        # Declare the arguments to save the information about the simulation:
        self.declare_parameter('fog_type', 'exponential')
        self.declare_parameter('fog_density', 0.01)
        self.declare_parameter('camera_noise_type', 'Gaussian')
        self.declare_parameter('camera_noise_std', 0.01)
        self.declare_parameter('camera_resolution',"3776x2360")
        self.declare_parameter('clutter',"Hanscom_Air_Force_Base_Sourrandings")
        self.declare_parameter('Yolo_model',"Yolo_n")
        self.declare_parameter('Frontal_camera_active',"True")
        self.declare_parameter('Frontal_rigth_camera_active',"True")
        self.declare_parameter('Back_rigth_camera_active',"True")
        self.declare_parameter('Back_left_camera_active',"True")
        self.declare_parameter('Frontal_left_camera_active',"True")

        # Declare arguments to show the ADS-B working:
        self.declare_parameter('plot_adsb_diff', 0)

        # Save the arguments in variables:
        self.waypoint_list = self.get_parameter('waypoint_list').value
        self.tcpa_list = self.get_parameter('tcpa_list').value
        self.own_airplane = self.get_parameter('own_airplane').value
        self.in_airplane = self.get_parameter('in_airplane').value
        self.save_dir = self.get_parameter('data_directory').value
        self.frequency = self.get_parameter('frecuency').value
        self.fog_type = self.get_parameter('fog_type').value
        self.fog_density = self.get_parameter('fog_density').value
        self.camera_noise_type = self.get_parameter('camera_noise_type').value
        self.camera_noise_standard_deviation = self.get_parameter('camera_noise_std').value
        self.camera_resultion = self.get_parameter('camera_resolution').value
        self.clutter = self.get_parameter('clutter').value
        self.YOLO_model = self.get_parameter('Yolo_model').value
        self.frontal_camera_active = self.get_parameter('Frontal_camera_active').value
        self.frontal_rigth_camera_active = self.get_parameter('Frontal_rigth_camera_active').value
        self.frontal_left_camera_active = self.get_parameter('Frontal_left_camera_active').value
        self.back_rigth_camera_active = self.get_parameter('Back_rigth_camera_active').value
        self.back_left_camera_active = self.get_parameter('Back_left_camera_active').value
        self.adsb_bool = self.get_parameter('plot_adsb_diff').value
        if self.adsb_bool > 0:
            self.plot_adsb_diff = True
        else:
            self.plot_adsb_diff = False

        # Gnerate the set of Waypoints for both airplanes;
        # Divide the encunter set numbers in a list
        self.waypoint_indices = list(map(int, self.waypoint_list.split(',')))
        self.get_logger().info(f"The MIT encoutner sets that are simulate are: {self.waypoint_indices}")
        # Divide the tcpa for each encounter set:
        self.tcpa_value = list(map(float,self.tcpa_list.split(',')))
        self.get_logger().info(f"The time ogf CPA are: {self.tcpa_value}")
        # Save a list with the Trakjectories as a string for the executable and float values for plotting
        # Ownship:
        self.own_traj = []
        self.own_traj_data = []
        # Intruder
        self.in_traj = []
        self.in_traj_data = []
        # Define the simulation time you want to simualte:
        sim_time  = 75
        # Strating row and finishig row to do the simulation:
        self.initial_start_time = {}
        # Define the data from the encuenter information:
        for index, tcpa in zip(self.waypoint_indices, self.tcpa_value):
            # Deine the number of enounter folder
            folder_num = math.floor((index-1)/1000)
            folder_name = f"encounters_{folder_num:03d}001"

            # Intruder csv file:
            in_csv_name = f"intstates_{index:06d}.csv"
            in_encounter_path = os.path.join(
                "/home/adcl/AirplanePathFollower/src/plane_follow_exec/DATA", ##### IMPORTANT CHANGE THE DIRECTORY WHERE YOU SAVE THE ENCOUNTER SET
                "terminal_encounter_state_data_20200630",
                folder_name,
                in_csv_name
            )
            # Read the intruder csv file:
            df_intruder = pd.read_csv(in_encounter_path)
            # Deinfe the inital time:
            init_time = max(0.0, tcpa - float(random.randint(25, 50)))
            self.initial_start_time[index] = init_time
            # Define the final time:
            final_time =  min(init_time + sim_time, len(df_intruder) - 1)
            df_intruder = df_intruder.iloc[int(init_time): int(final_time) + 1].reset_index(drop=True)
            # Save it as a srting to send it to the executable:
            in_north = (df_intruder[' y_ft'] * 0.3048).tolist()
            in_east = (df_intruder[' x_ft'] * 0.3048).tolist()
            in_down = (df_intruder[' alt_ft'] * -0.3048).tolist()
            in_cmdVel = (df_intruder[' speed_ftps'] * 0.3048).tolist()
            in_traj_string = ";".join(f"{in_north[i]},{in_east[i]},{in_down[i]},{in_cmdVel[i]}" for i in range(len(in_east)))
            self.in_traj.append(in_traj_string)
            # Save it as numbers to plot the trajectories during the simulations:
            in_traj_array = np.column_stack([
                df_intruder[' y_ft'].values * 0.3048,
                df_intruder[' x_ft'].values * 0.3048,
                df_intruder[' alt_ft'].values * 0.3048,
                df_intruder[' speed_ftps'].values * 0.3048,
                df_intruder[' roll_rad'],
                df_intruder[' pitch_rad'],
                np.pi/2 - df_intruder[' relhdg_rad'],
            ])  
            self.in_traj_data.append(in_traj_array)

            # Ownship csv file:
            own_csv_name = f"ownstates_{index:06d}.csv"
            own_encounter_path = os.path.join(
                "/home/adcl/AirplanePathFollower/src/plane_follow_exec/DATA", # IMPORTANT: Change this lane with respect to the folder witht eh trajectories.
                "terminal_encounter_state_data_20200630",
                folder_name,
                own_csv_name
            )
            # Read the ownship csv file:
            df_ownship= pd.read_csv(own_encounter_path)
            df_ownship =df_ownship.iloc[int(init_time): int(final_time) + 1].reset_index(drop=True)
            # Save the coordiantes in a folrmat [north,east,down,cmd_vel]
            own_north = (df_ownship[' y_ft'] * 0.3048).tolist()
            own_east = (df_ownship[' x_ft'] * 0.3048).tolist()
            own_down = (df_ownship[' alt_ft'] * -0.3048).tolist()
            own_cmdVel = (df_ownship[' speed_ftps'] * 0.3048).tolist()
            own_traj_string = ";".join(f"{own_north[i]},{own_east[i]},{own_down[i]},{own_cmdVel[i]}" for i in range(len(own_east)))
            self.own_traj.append(own_traj_string)
            # Sav ethe trajectories to plot them:
            own_traj_array = np.column_stack([
                df_ownship[' y_ft'].values * 0.3048,
                df_ownship[' x_ft'].values * 0.3048,
                df_ownship[' alt_ft'].values * 0.3048,
                df_ownship[' speed_ftps'].values * 0.3048,
                df_ownship[' roll_rad'],
                df_ownship[' pitch_rad'],
                np.pi/2 - df_ownship[' relhdg_rad'],
            ])
            self.own_traj_data.append(own_traj_array)
        
        self.get_logger().info(f"Initial times {self.initial_start_time}")
        

        # Start the plot as teh Airplane trajectory:
        plt.ion()
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        plt.show(block=False)

        # QoS reliable to amtch with the c++ states:
        q_reliable = QoSProfile(depth=10)
        q_reliable.reliability = ReliabilityPolicy.RELIABLE
        q_reliable.durability  = DurabilityPolicy.VOLATILE

        # QoS for the events and flags during the simulation:
        q_events = QoSProfile(depth=1)
        q_events.reliability = ReliabilityPolicy.RELIABLE
        q_events.durability  = DurabilityPolicy.VOLATILE

        # Create subscribers to see whenver one aiplne is near the goal:
        self.own_traj_complete_sub =self.create_subscription(Bool,f"/{self.own_airplane}/traj_complete",self.own_complete_callback,q_events)
        self.in_traj_complete_sub =self.create_subscription(Bool,f"/{self.in_airplane}/traj_complete",self.in_complete_callback,q_events)

        # Create subscribers tot eh adsb_out of the intruder inc ase it want to be plot:
        self.adbs_int_info = self.create_subscription(AdsbData,f"/{self.in_airplane}/adsb_out",self.adsb_callback,q_reliable)
        # Save the states of the adsb
        self.adsb_x_ft = []
        self.adsb_y_ft = []
        self.adsb_z_ft = []
        self.adsb_vx_fts = []
        self.adsb_vy_fts = []
        self.adsb_vz_fts = []
        # Create variables to save the individual information:
        self.in_adsb_x = 0
        self.in_adsb_y = 0
        self.in_adsb_z = 0
        self.in_adsb_vx = 0
        self.in_adsb_vy = 0
        self.in_adsb_vz = 0
        # Check if adsb at least has one value:
        self.has_adsb = False
        
        if self.plot_adsb_diff:
            # Create lists to save the adsb_information:
            self.time_adsb = []
            # List to get the erros of the states:
            self.adsb_err_x_ft = []
            self.adsb_err_y_ft = []
            self.adsb_err_z_ft = []
            self.adsb_err_vx_fts = []
            self.adsb_err_vy_fts = []
            self.adsb_err_vz_fts = []


        # Generate lists to save previous states of the airplane to plot them:
        # Ownship
        self.own_airplane_states_sub_ = self.create_subscription(AirplaneKineticModelInfo,f"/{self.own_airplane}/states",self.own_states_callback,q_reliable)
        self.own_states_north = []
        self.own_states_east = []
        self.own_states_alt = []
        # Intruder:
        self.in_airplane_states_sub_ = self.create_subscription(AirplaneKineticModelInfo,f"/{self.in_airplane}/states",self.in_states_callback,q_reliable)
        self.in_states_north = []
        self.in_states_east = []
        self.in_states_alt = []

        # Publish when one is near the goal:
        qos_2 = QoSProfile(depth=1)
        qos_2.reliability = ReliabilityPolicy.RELIABLE
        qos_2.durability = DurabilityPolicy.VOLATILE   
        self.complete_trajectory_pub_ = self.create_publisher(Bool, 'complete_encounter', qos_2)

        # Generate a service to change teh position adn spawn the models in different positions:
        self.move_the_airplane_client = self.create_client(SetEntityState, '/gazebo/set_entity_state') # Remember to add the gazebo_ros_state plugin to the world to make it work

        # Topic to close the simulation where all the sets are completed:
        self.stop_pub = self.create_publisher(Bool,'/stop_simulation',1)

        # Subscriber of the YOLO detection system:
        self.airplane_recog_sub = self.create_subscription(
            Yolov11Inference360,
            f"/{self.own_airplane}/YOLOv11_inference",
            self.score_airplane_callback,
            10
        )

        # Generate teh clients to start and stop the YOLO detection: 
        self.start_detection_client = self.create_client(Trigger, f"{self.own_airplane}_start_detection") 
        self.stop_detection_client = self.create_client(Trigger, f"{self.own_airplane}_stop_detection") 

        ######### Section done to save the simulation information in teh ENU axes#########
        # Create the timer
        self.timer = self.create_timer(1/self.frequency,self.timer_callback)

        # Create and restart the lists to save the information using the timer:
        self.restart_saving_lists()


    def restart_saving_lists(self):
         # flag to divide when each simulation starts:
        self.own_ready = False
        self.in_ready = False
        self.count = 0
        self.init_real_time = 0
        self.init_sim_time = 0
        # Individual values:
        self.real_time = 0
        self.sim_time = 0
        # Ownship:
        self.own_x = 0
        self.own_y = 0
        self.own_alt = 0
        self.own_vx = 0
        self.own_vy = 0
        self.own_vz = 0
        self.own_roll = 0
        self.own_pitch = 0
        self.own_yaw = 0
        # Intruder:
        self.in_x = 0
        self.in_y = 0
        self.in_alt = 0
        self.in_vx = 0
        self.in_vy = 0
        self.in_vz = 0
        self.in_roll = 0
        self.in_pitch = 0
        self.in_yaw = 0
        # Dtection:
        self.front_camera_score = 0
        self.rigth_front_camera_score = 0
        self.rigth_back_camera_score = 0
        self.left_back_camera_score = 0
        self.left_front_camera_score = 0
        self.preprocess_time = 0
        self.inference_time = 0
        self.postprocess_time = 0

        # Create lists to save the information using the timer:
        # time:
        self.real_time_s = []
        self.sim_time_s = []
        self.rtf = []
        # Ownship infromation
        self.own_x_ft = []
        self.own_y_ft = []
        self.own_alt_ft = []
        self.own_vx_fts = []
        self.own_vy_fts = []
        self.own_vz_fts = []
        self.own_roll_rad = []
        self.own_pitch_rad = []
        self.own_yaw_rad = []
        # Intruder information:
        self.in_x_ft = []
        self.in_y_ft = []
        self.in_alt_ft = []
        self.in_vx_fts = []
        self.in_vy_fts = []
        self.in_vz_fts = []
        self.in_roll_rad = []
        self.in_pitch_rad = []
        self.in_yaw_rad = []
        # Ownship camera information:
        self.front_camera_scores = []
        self.rigth_front_camera_scores = []
        self.rigth_back_camera_scores = []
        self.left_back_camera_scores = []
        self.left_front_camera_scores = []
        self.preprocess_times = []
        self.inference_times = []
        self.postprocess_times = []

        # In case the adsb need to be plot re-initialize the variables:
        self.in_adsb_x = 0
        self.in_adsb_y = 0
        self.in_adsb_z = 0
        self.in_adsb_vx = 0
        self.in_adsb_vy = 0
        self.in_adsb_vz = 0
        self.in_adsb_course = 0
        self.has_adsb = False
        self.time_adsb = []
        self.adsb_x_ft = []
        self.adsb_y_ft = []
        self.adsb_z_ft = []
        self.adsb_vx_fts = []
        self.adsb_vy_fts = []
        self.adsb_vz_fts = []
        if self.plot_adsb_diff:
            self.adsb_err_x_ft = []
            self.adsb_err_y_ft = []
            self.adsb_err_z_ft = []
            self.adsb_err_vx_fts = []
            self.adsb_err_vy_fts = []
            self.adsb_err_vz_fts = []


    
    # Define a call service to start the detection or stop it:
    def call_service_Trigger(self, client, service_name: str) -> bool:
        req = Trigger.Request()
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None and future.result().success:
            self.get_logger().info(f'{service_name} service call succeeded: {future.result().message}')
            return True
        else:
            self.get_logger().info(f'{service_name} service call failed')


    # Detect when the Airplane its near the goal position:
    def own_complete_callback(self,msg):
        if msg.data and not getattr(self, "traj_complete", False):
            self.traj_complete = True
            self.complete_trajectory_pub_.publish(Bool(data=True))
            self.get_logger().info("Ownship completed the trajectory so closing intruder")
    def in_complete_callback(self,msg):
        if msg.data and not getattr(self, "traj_complete", False):
            self.traj_complete = True
            self.complete_trajectory_pub_.publish(Bool(data=True))
            self.get_logger().info("Intruder completed the trajectory so closing ownship")


    # States callback to save it in a list and plot them:
    def own_states_callback(self,msg):
        if not hasattr(self, "line_states_own"):
            return
        if msg.velocity_a > 0:
            # Analize if simulation started:
            self.own_ready = True
            if self.count == 0:
                self.count = 1
                self.init_real_time = msg.real_time
                self.init_sim_time = msg.sim_time
            # Save the info for the timer:
            # time
            self.real_time = msg.real_time
            self.sim_time = msg.sim_time
            # Position:
            self.own_y = msg.north/0.3048
            self.own_x = msg.east/0.3048
            self.own_alt = msg.up/0.3048
            # Orientation:
            self.own_roll = msg.roll
            self.own_pitch = -msg.fpa
            self.own_yaw = np.pi/2-msg.course
            # Velocity
            vel_body_x = msg.velocity_a/0.3048
            self.own_vx = vel_body_x*np.cos(self.own_yaw )*np.cos(self.own_pitch)
            self.own_vy = vel_body_x*np.sin(self.own_yaw )*np.cos(self.own_pitch)
            self.own_vz = -vel_body_x*np.sin(self.own_pitch)

            # Plot the path in Python:
            self.own_states_north.append(self.own_y)
            self.own_states_east.append(self.own_x)
            self.own_states_alt.append(self.own_alt)
            self.line_states_own.set_data(self.own_states_east, self.own_states_north)
            self.line_states_own.set_3d_properties(self.own_states_alt)
            self.ax.relim()
            self.ax.autoscale_view()
            plt.draw()
            plt.pause(0.001)


    def in_states_callback(self,msg):
        if not hasattr(self, "line_states_in"):
            return
        if msg.velocity_a > 0:
            # Analize if simulation started:
            self.in_ready = True
            if self.count == 0:
                self.count = 1
                self.init_real_time = msg.real_time
                self.init_sim_time = msg.sim_time
                
            # Save the info for the timer:
            # time
            self.real_time = msg.real_time
            self.sim_time = msg.sim_time
            # Position:
            self.in_y = msg.north/0.3048
            self.in_x = msg.east/0.3048
            self.in_alt = msg.up/0.3048
            # Orientation:
            self.in_roll = msg.roll
            self.in_pitch = -msg.fpa
            self.in_yaw = np.pi/2-msg.course
            # Velocity
            vel_body_x = msg.velocity_a/0.3048
            self.in_vx = vel_body_x*np.cos(self.in_yaw )*np.cos(self.in_pitch)
            self.in_vy = vel_body_x*np.sin(self.in_yaw )*np.cos(self.in_pitch)
            self.in_vz = -vel_body_x*np.sin(self.in_pitch)

            # Plot the path in Python:
            self.in_states_north.append(self.in_y)
            self.in_states_east.append(self.in_x)
            self.in_states_alt.append(self.in_alt)
            self.line_states_in.set_data(self.in_states_east, self.in_states_north)
            self.line_states_in.set_3d_properties(self.in_states_alt)
            self.ax.relim()
            self.ax.autoscale_view()
            plt.draw()
            plt.pause(0.001)


    # Funciton to spawn the airplanes in a position and with an orientation:
    def call_service_change_position(self, client, service_name: str, entity_name: str, east: float, north: float, alt: float, roll: float, fpa: float, course: float) -> bool:
        # Set the model name and the posiiton
        self.get_logger().info(f'{service_name} change service started')
        req = SetEntityState.Request()
        req.state.name = entity_name
        req.state.pose.position.x = east
        req.state.pose.position.y = north
        req.state.pose.position.z = alt
        # Transform the euler angels into quaternions:
        yaw = np.pi/2 - course
        pitch = -fpa
        quat = R.from_euler('xyz',[roll,pitch,yaw]).as_quat()
        req.state.pose.orientation.x = quat[0]
        req.state.pose.orientation.y = quat[1]
        req.state.pose.orientation.z = quat[2]
        req.state.pose.orientation.w = quat[3]
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None and future.result().success:
            self.get_logger().info(f'{service_name} service call succeeded')
            return True
        else:
            self.get_logger().info(f'{service_name} service call failed')
            return False
        
    
    # Setup the plot incase adsb need to be comapred:
    def setup_diff_plot(self, waypoint_idx):
        if not self.plot_adsb_diff:
            return
        plt.ion()
        self.fig_diff, self.ax_diff = plt.subplots(3, 2, figsize=(10, 8), constrained_layout=True)
        self.fig_diff.canvas.manager.set_window_title(f"ADS-B vs State Δ | Encounter {waypoint_idx}")

        # Axes order: x,y,z,vx,vy,vz
        titles = [r'Δx (ft)', r'Δ$v_x$ (ft/s)', r'Δy (ft)', r'Δ$v_y$ (ft/s)', r'Δz (ft)', r'Δ$v_z$ (ft/s)']
        self.diff_lines = []
        for ax, title in zip(self.ax_diff.flatten(), titles):
            ax.set_title(title)
            ax.set_xlabel('Sim time (s)')
            ax.grid(True)
            (line,) = ax.plot([], [])
            self.diff_lines.append(line)

        # handy refs:
        (self.ax_dx, self.ax_dvx, self.ax_dy, self.ax_dvy, self.ax_dz, self.ax_dvz) = self.ax_diff.flatten()
        

    # Ploting system to restart the ploting and plot the new trajectories every timie it completes one wncounter set:
    def setup_plot(self,in_traj,own_traj,waypoint_idx):
        for _ in range(5):
            rclpy.spin_once(self, timeout_sec=0.1)
        
        self.ax.clear()
        self.ax.set_xlabel('East (ft)')
        self.ax.set_ylabel('North (ft)')
        self.ax.set_zlabel('Altitude (ft)')
        self.ax.set_title(f'UAV Waypoint follower Monitoring {waypoint_idx}')

        # Convert the values in feets:
        own_e_ft = own_traj[:, 1] / 0.3048
        own_n_ft = own_traj[:, 0] / 0.3048
        own_alt_ft = own_traj[:, 2] / 0.3048
        in_e_ft  = in_traj[:, 1] / 0.3048
        in_n_ft  = in_traj[:, 0] / 0.3048
        in_alt_ft = in_traj[:, 2] / 0.3048

        # Plot the waypoints:
        self.ax.plot(own_e_ft, own_n_ft, own_alt_ft, 'ro--', alpha = 0.9, label='Ownship Waypoints')
        self.ax.plot(in_e_ft, in_n_ft, in_alt_ft, 'go--', alpha = 0.9, label='Intruder Waypoints')


        # Generate the dynamics plot for the states as a placeholder:
        (self.line_states_own,) = self.ax.plot([],[],[], 'b', label = 'Ownship path')
        (self.line_states_in,) = self.ax.plot([],[],[], 'm', label = 'Intruder path')
        self.ax.legend()

        # Find teh limits of the Waypoints to do an axis equal:
        e_all = np.concatenate([own_e_ft,in_e_ft])
        n_all = np.concatenate([own_n_ft,in_n_ft])
        alt_all = np.concatenate([own_alt_ft,in_alt_ft])
        max_range = max( np.ptp(e_all),np.ptp(n_all),np.ptp(alt_all)) / 2.0
        mid_x = np.mean(e_all)
        mid_y = np.mean(n_all)
        mid_z = np.mean(alt_all)
        self.ax.set_xlim(mid_x - max_range, mid_x + max_range)
        self.ax.set_ylim(mid_y - max_range, mid_y + max_range)
        self.ax.set_zlim(mid_z - max_range, mid_z + max_range)
        plt.show()

        # Setup the plots of the differnce between the adsb states and real states:
        self.setup_diff_plot(waypoint_idx)

            

    

    # Save the scores saved depending on the camera that detects it
    def score_airplane_callback(self, msg):
        self.front_camera_score = 0
        self.rigth_front_camera_score = 0
        self.rigth_back_camera_score = 0
        self.left_back_camera_score = 0
        self.left_front_camera_score = 0
        self.preprocess_time = msg.preprocess_time
        self.inference_time = msg.inference_time
        self.postprocess_time = msg.postprocess_time
        if not msg.yolov11_inference:
            self.preprocess_time = 0
            self.inference_time = 0
            self.postprocess_time = 0
        else:
            for yolov11_inf in msg.yolov11_inference:
                if yolov11_inf.camera_name == "frontal_camera" and yolov11_inf.score > self.front_camera_score:
                    self.front_camera_score = yolov11_inf.score
                elif yolov11_inf.camera_name == "rigth_front_camera" and yolov11_inf.score > self.rigth_front_camera_score:
                    self.rigth_front_camera_score = yolov11_inf.score
                elif yolov11_inf.camera_name == "rigth_back_camera" and yolov11_inf.score > self.rigth_back_camera_score:
                    self.rigth_back_camera_score = yolov11_inf.score
                elif yolov11_inf.camera_name == "left_back_camera" and yolov11_inf.score > self.left_back_camera_score:
                    self.left_back_camera_score = yolov11_inf.score
                elif yolov11_inf.camera_name == "left_front_camera" and yolov11_inf.score > self.left_front_camera_score:
                    self.left_front_camera_score = yolov11_inf.score

    
    # Save the adsb_info to be plot:
    def adsb_callback(self, msg):
        # Get the last values:
        self.in_adsb_x = msg.east/0.3048
        self.in_adsb_y = msg.north/0.3048
        self.in_adsb_z = msg.up/0.3048
        self.in_adsb_vx = msg.v_east/0.3048
        self.in_adsb_vy = msg.v_north/0.3048
        self.in_adsb_vz = msg.v_up/0.3048
        self.in_adsb_course = msg.course

        # Chaeck that adsb at least has one value:
        self.has_adsb = True


    # Timer callback:
    def timer_callback(self):
        if not (self.in_ready and self.own_ready):
            return
        # Time:
        self.real_time_s.append(self.real_time - self.init_real_time)
        self.sim_time_s.append(self.sim_time - self.init_sim_time)
        # Real time factor:
        if (self.real_time - self.init_real_time) == 0:
            self.rtf.append(0)
        else:
            self.rtf.append((self.sim_time - self.init_sim_time)/(self.real_time - self.init_real_time))
        # Ownship:
        self.own_x_ft.append(self.own_x)
        self.own_y_ft.append(self.own_y)
        self.own_alt_ft.append(self.own_alt)
        self.own_vx_fts.append(self.own_vx)
        self.own_vy_fts.append(self.own_vy)
        self.own_vz_fts.append(self.own_vz)
        self.own_roll_rad.append(self.own_roll)
        self.own_pitch_rad.append(self.own_pitch)
        self.own_yaw_rad.append(self.own_yaw)
        # Intruder:
        self.in_x_ft.append(self.in_x)
        self.in_y_ft.append(self.in_y)
        self.in_alt_ft.append(self.in_alt)
        self.in_vx_fts.append(self.in_vx)
        self.in_vy_fts.append(self.in_vy)
        self.in_vz_fts.append(self.in_vz)
        self.in_roll_rad.append(self.in_roll)
        self.in_pitch_rad.append(self.in_pitch)
        self.in_yaw_rad.append(self.in_yaw)
        # Detection model:
        self.preprocess_times.append(self.preprocess_time)
        self.inference_times.append(self.inference_time)
        self.postprocess_times.append(self.postprocess_time)
        self.front_camera_scores.append(self.front_camera_score)
        self.rigth_front_camera_scores.append(self.rigth_front_camera_score)
        self.left_front_camera_scores.append(self.left_front_camera_score)
        self.rigth_back_camera_scores.append(self.rigth_back_camera_score)
        self.left_back_camera_scores.append(self.left_back_camera_score)
        
        # Save the ADSB-DATA only when it wants to be plottes:
        self.time_adsb.append(self.sim_time - self.init_sim_time)
        # Save the data of the adsb in the lsits:
        self.adsb_x_ft.append(self.in_adsb_x)   
        self.adsb_y_ft.append(self.in_adsb_y)        
        self.adsb_z_ft.append(self.in_adsb_z)        
        self.adsb_vx_fts.append(self.in_adsb_vx)        
        self.adsb_vy_fts.append(self.in_adsb_vy)        
        self.adsb_vz_fts.append(self.in_adsb_vz) 

        if self.plot_adsb_diff and self.has_adsb:                
            # Use latest samples (everything is in feet / ft/s)
            dx  = self.in_x_ft[-1]   - self.in_adsb_x
            dy  = self.in_y_ft[-1]   - self.in_adsb_y
            dz  = self.in_alt_ft[-1] - self.in_adsb_z
            dvx = self.in_vx_fts[-1] - self.in_adsb_vx
            dvy = self.in_vy_fts[-1] - self.in_adsb_vy
            dvz = self.in_vz_fts[-1] - self.in_adsb_vz

            # Save the errors in a list
            self.adsb_err_x_ft.append(dx)
            self.adsb_err_y_ft.append(dy)
            self.adsb_err_z_ft.append(dz)
            self.adsb_err_vx_fts.append(dvx)
            self.adsb_err_vy_fts.append(dvy)
            self.adsb_err_vz_fts.append(dvz)

            # update live lines vs sim time
            series = [self.adsb_err_x_ft, self.adsb_err_vx_fts, self.adsb_err_y_ft, self.adsb_err_vy_fts, self.adsb_err_z_ft,  self.adsb_err_vz_fts]
            for line, data in zip(self.diff_lines, series):
                line.set_data(self.time_adsb, data)

            # rescale axes
            for ax2 in self.ax_diff.flatten():
                ax2.relim()
                ax2.autoscale_view()

            plt.pause(0.001)


    
    # Save the inframtion in the diferent folders:
    def save_information(self, waypoint_idx):
        # Create the folder of the fligth:
        fligth_folder = os.path.join(self.save_dir,f"Encounter_{waypoint_idx}")
        os.makedirs(fligth_folder)

        # Generate the csv files for the different information:
        # Airplane states:
        folder_airplane_states = os.path.join(fligth_folder, "states_data")
        os.makedirs(folder_airplane_states)
        # Ownship:
        ownship_states_file = os.path.join(folder_airplane_states, "ownship_states.csv")
        with open(ownship_states_file, 'w') as file:
            file.write("Real_time_s,Simulation_time_s,Real_time_factor,x_ft,y_ft,alt_ft,vx_fps,vy_fps,vz_fps,roll_rad,pitch_rad,yaw_rad\n")
            for i in range(len(self.rtf)):
                file.write(f"{self.real_time_s[i]},{self.sim_time_s[i]},{self.rtf[i]},{self.own_x_ft[i]},{self.own_y_ft[i]},{self.own_alt_ft[i]},{self.own_vx_fts[i]},{self.own_vy_fts[i]},{self.own_vz_fts[i]},{self.own_roll_rad[i]},{self.own_pitch_rad[i]},{self.own_yaw_rad[i]}\n")
        # Intruder:
        intruder_states_file = os.path.join(folder_airplane_states, "intruder_states.csv")
        with open(intruder_states_file, 'w') as file:
            file.write("Real_time_s,Simulation_time_s,Real_time_factor,x_ft,y_ft,alt_ft,vx_fps,vy_fps,vz_fps,roll_rad,pitch_rad,yaw_rad\n")
            for i in range(len(self.rtf)):
                file.write(f"{self.real_time_s[i]},{self.sim_time_s[i]},{self.rtf[i]},{self.in_x_ft[i]},{self.in_y_ft[i]},{self.in_alt_ft[i]},{self.in_vx_fts[i]},{self.in_vy_fts[i]},{self.in_vz_fts[i]},{self.in_roll_rad[i]},{self.in_pitch_rad[i]},{self.in_yaw_rad[i]}\n")
        # ADS-B Intruder Data:
        adsb_data_file = os.path.join(folder_airplane_states, "adsb_intruder_states.csv")
        with open(adsb_data_file, 'w') as file:
            file.write("Simulation_time_s,x_ft,y_ft,alt_ft,vx_fps,vy_fps,vz_fps\n")
            for i in range(len(self.time_adsb)):
                file.write(f"{self.time_adsb[i]},{self.adsb_x_ft[i]},{self.adsb_y_ft[i]},{self.adsb_z_ft[i]},{self.adsb_vx_fts[i]},{self.adsb_vy_fts[i]},{self.adsb_vz_fts[i]}\n")

        
        # Generate the csv file with the YOLO detection summary;
        folder_detection = os.path.join(fligth_folder, "detection_data")
        os.makedirs(folder_detection)
        # Detection Confidence;
        detection_summary = os.path.join(folder_detection, "Detection_process.csv")
        with open(detection_summary, 'w') as file:
            file.write("Real_time_s,Simulation_time_s,Real_time_factor,Range_ft,Ownship_Course_rad,Intruder_course_rad,Camera1_Confidence_level,Camera2_Confidence_level,Camera3_Confidence_level,Camera4_Confidence_level,Camera5_Confidence_level,Preprocess_time_ms,Inference_time_ms,Postprocess_time_ms\n")
            for i in range(len(self.rtf)):
                range_ft = np.sqrt((self.own_x_ft[i]-self.in_x_ft[i])**2+(self.own_y_ft[i]-self.in_y_ft[i])**2+(self.own_alt_ft[i]-self.in_alt_ft[i])**2)
                own_course = np.pi/2-self.own_yaw_rad[i]
                in_course = np.pi/2-self.in_yaw_rad[i]
                file.write(f"{self.real_time_s[i]},{self.sim_time_s[i]},{self.rtf[i]},{range_ft},{own_course},{in_course},{self.front_camera_scores[i]},{self.rigth_front_camera_scores[i]},{self.rigth_back_camera_scores[i]},{self.left_back_camera_scores[i]},{self.left_front_camera_scores[i]},{self.preprocess_times[i]},{self.inference_times[i]},{self.postprocess_times[i]}\n")

        # Generate a csv file with the information about the simulation:
        file_detection_info = os.path.join(folder_detection, 'Detection_events.csv')
        with open(file_detection_info,'w') as file:
            file.write(f"Fog_type:,{self.fog_type}\n")
            file.write(f"Fog_density:,{self.fog_density}\n")
            file.write(f"Camera_noise_type:,{self.camera_noise_type}\n")
            file.write(f"Camera_noise_std:,{self.camera_noise_standard_deviation}\n")
            file.write(f"Camera_resolution:,{self.camera_resultion}\n")
            file.write(f"Clutter:,{self.clutter}\n")
            file.write(f"YOLO_model:,{self.YOLO_model}\n")
            file.write(f"Initial time:,{self.initial_start_time.get(waypoint_idx, '')}\n")
            file.write(f"Frontal_camera_active:,{self.frontal_camera_active}\n")
            file.write(f"Frontal_rigth_camera_active:,{self.frontal_rigth_camera_active}\n")
            file.write(f"Back_rigth_camera_active:,{self.back_rigth_camera_active}\n")
            file.write(f"Back_left_camera_active:,{self.back_left_camera_active}\n")
            file.write(f"Frontal_left_camera_active:,{self.frontal_left_camera_active}\n")

    # Function to do the following of he encounter sets:
    def mit_encounter_waypoint_follower(self):
        if not self.call_service_Trigger(self.start_detection_client, f"{self.own_airplane}_start_detection"):
            return
        # Iterate in all the encunters you submit in the list:
        for i, (in_encounter,own_encounter, in_trajectory, own_trajectory) in enumerate(zip(self.in_traj, self.own_traj, self.in_traj_data, self.own_traj_data)):
            
            # Wait some time before starting the path following:
            self.restart_saving_lists()
            time.sleep(5)

            # Start the plot:
            self.setup_plot(in_trajectory,own_trajectory,self.waypoint_indices[i])
            # Restrart the list with the past trajectories:
            self.traj_complete = False
            self.own_states_north = []
            self.own_states_east = []
            self.own_states_alt = []
            self.in_states_north = []
            self.in_states_east = []
            self.in_states_alt = []
            plt.draw()
            plt.pause(1.0)

            # Move both airplanes to the initial position:
            if not self.call_service_change_position(self.move_the_airplane_client, '/gazebo/set_entity_state',self.own_airplane, own_trajectory[0,1],own_trajectory[0,0],own_trajectory[0,2],own_trajectory[0,4],own_trajectory[0,5],own_trajectory[0,6]):
                        return
            if not self.call_service_change_position(self.move_the_airplane_client, '/gazebo/set_entity_state',self.in_airplane, in_trajectory[0,1],in_trajectory[0,0],in_trajectory[0,2],in_trajectory[0,4],in_trajectory[0,5],in_trajectory[0,6]):
                        return
            
            # Run both waypoint followers:
            process_own = subprocess.Popen(["ros2","run","airplane_gazebo_plugin","airplane_waypoint_follower",self.own_airplane,own_encounter])
            process_in = subprocess.Popen(["ros2","run","airplane_gazebo_plugin","airplane_waypoint_follower",self.in_airplane,in_encounter])
            
            # Start the following executables:
            try:
                start_own = self.create_client(Trigger, f"/{self.own_airplane}/trigger_service")
                start_in = self.create_client(Trigger, f"/{self.in_airplane}/trigger_service")
                # Create a client to start the adsb_det:
                start_in_adsb = self.create_client(Trigger, f"/{self.in_airplane}/adsb_start_service")
                stop_in_adsb = self.create_client(Trigger, f"/{self.in_airplane}/adsb_stop_service")

                for cli, name in [(start_own, "own"), (start_in, "intruder")]:
                    while not cli.wait_for_service(timeout_sec=1.0):
                        self.get_logger().warn(f"Waiting for {name} trigger service...")

                fut_own = start_own.call_async(Trigger.Request())
                fut_in = start_in.call_async(Trigger.Request())
                fut_start_in_adsb = start_in_adsb.call_async(Trigger.Request())
                # Spin both executables:
                rclpy.spin_until_future_complete(self, fut_start_in_adsb)
                rclpy.spin_until_future_complete(self, fut_own)
                rclpy.spin_until_future_complete(self, fut_in)
                # Wait until both finish;
                timeout = 300
                t0 = time.time()
                while time.time() - t0 < timeout:
                    rclpy.spin_once(self, timeout_sec=0.1)
                    if process_own.poll() is not None or process_in.poll() is not None:
                        break
                    plt.pause(0.001) 
                # In case the processes are running still clsoe them:
                if process_own.poll() is None:
                    process_own.terminate()
                if process_in.poll() is None:
                    process_in.terminate()
                # Make sure:
                try:
                    process_own.wait(timeout=10)
                    process_in.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    process_own.kill()
                    process_in.kill()
            finally:
                fut_stop_in_adsb = stop_in_adsb.call_async(Trigger.Request())
                rclpy.spin_until_future_complete(self, fut_stop_in_adsb)
                self.save_information(self.waypoint_indices[i])
                plt.pause(5)
                
        # Stop teh simulation when the process finish
        msg = Bool()
        if not self.call_service_Trigger(self.start_detection_client, f"{self.own_airplane}_stop_detection"):
                return
        msg.data = True
        self.stop_pub.publish(msg)
    


def main(args=None):
    rclpy.init(args=args)
    node = MITEncounterFollower()
    try:
        node.mit_encounter_waypoint_follower()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
