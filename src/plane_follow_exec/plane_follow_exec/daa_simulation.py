from cProfile import label
from enum import Flag
import os
import math
from queue import Empty
from tkinter import PROJECTING, S
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import time
import itertools

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
from airplane_gazebo_plugin.msg import AirplaneKineticModelInfo, AvoidanceStates, AdsbData
from custom_msgs.msg import Yolov11Inference360
# Service to spawn the airplane in a position 
from gazebo_msgs.srv import SetEntityState

# Library to start process on the command window:
import subprocess
import os 
import contextlib
import signal
import atexit


# Function to pass the waypoints from a string to ana array:
def parse_wp_string(wp_string):
    # To make it work the Waypoints would need to have the next format "North, East, Down, Overall Speed" in feets at each waypoint:"
    rows =[]
    for wp in wp_string.split(';'):
        wp = wp.strip()
        if not wp:
            continue
        nums = [float(x) for x in wp.split(',')]
        rows.append(nums)
    parsed = np.array(rows, dtype=float)
    
    # Now save the waypoints so each element is in a column
    return np.column_stack([
        parsed[:,0] * 0.3048,   # N (m)
        parsed[:,1] * 0.3048,   # E (m)
        parsed[:,2] * -0.3048,  # Altitude (m)
        parsed[:,3] * 0.3048,   # Speed (m/s)
    ])


# Parse multiple waypoint strings from the overall string:
def parse_multiple_wp_strings(mult_wp_string):
    # This would split the different waypoint strings divided by % in the overall string of the UAVs
    # and save the inforamtion in numerical arrays lists to eb inputed in the dictionaries
    segments = []
    for seg in mult_wp_string.split('%'):
        seg = seg.strip()
        if not seg:
            continue
        segments.append(parse_wp_string(seg))
    return segments

# Format string from numbers:
def _format_num(x: float) -> str:
    # neat, short float formatting (no trailing zeros)
    s = f"{x:.6f}".rstrip('0').rstrip('.')
    return s if s else "0"


# Function to separate the multiple waypoints in a list of strings:
def separate_wp(mult_wp_string):
    waypoints_strings = []
    for wps in mult_wp_string.split('%'):
        wps = wps.strip()
        # Pass from ft to meters:
        out_wps = []
        for wp in wps.split(';'):
            wp = wp.strip()
            parts = [p.strip() for p in wp.split(',')]
            N_ft, E_ft, D_ft, V_ftps = map(float, parts)
            # Update it to meters:
            N_m = N_ft * 0.3048
            E_m = E_ft * 0.3048
            Down_m = D_ft * 0.3048
            Vel_mps = V_ftps * 0.3048

            # Save in out_waypooints as a new list:
            out_wps.append(f"{_format_num(N_m)}, {_format_num(E_m)}, {_format_num(Down_m)}, {_format_num(Vel_mps)}")
        
        # Save the modified waypoint strings:
        waypoints_strings.append('; '.join(out_wps))
    return waypoints_strings


# Generate the color-markers for the intruders:
def generate_inturders_color_maps(names):
    palette = plt.cm.get_cmap('tab20').colors
    ls_cycle = ['--','-',':']
    color_map = {}
    count = 0
    for name in names:
        color = palette[count % len(palette)]
        line_stl = ls_cycle[count % len(ls_cycle)]
        color_map[name] = {"color": color, "line_style" : line_stl}
        count += 1
    return color_map
    



# Start the executable:
class DAASimulation(Node):
    
    def __init__(self):
        # Generate the node:
        super().__init__('daa_simulation')

        # FOdler to savet he arplanes info:
        self.declare_parameter('data_directory', '/home/adcl/AirplanePathFollower/DATA/DAA_Fligths') # Folder to save the inforamtion
        self.save_dir = self.get_parameter('data_directory').value

        # Plot Characteristic
        self.intruder_lines = {}

        # Flag to save the ADS-B related info
        self.declare_parameter('save_adsb_info', False)
        self.save_adsb_info = self.get_parameter('save_adsb_info').value  

        # Declare the arguments of the avoider:
        self.declare_parameter('avoider_waypoints','0,50,-50,5;450,85,-20,5 % 500,100,-20,6;600,120,-10,6')
        self.declare_parameter('avoider_name','airplane_1')
        # Save the arguments of teh avoider:
        self.avoider_wp_string = separate_wp(self.get_parameter('avoider_waypoints').value)
        self.avoider_wp_info = parse_multiple_wp_strings(self.get_parameter('avoider_waypoints').value)
        self.avoider_name = self.get_parameter('avoider_name').value

        # Open the intruders argumetns and save them:
        # Find the number of intruders
        self.declare_parameter('number_intruders', '1')
        self.num_intruders = int(self.get_parameter('number_intruders').value)
        # Save the names and the waypoints string sin a dictionary
        self.intruders_info_string = {}
        # Save the waypoints as arrays in other 
        self.intruders_info = {}
        for i in range(self.num_intruders):
            self.declare_parameter(f'intruder_name_{i}', f'airplane_{i+2}')
            self.declare_parameter(f'intruder_waypoints_{i}', '0,50,-50,5;450,85,-20,5 % 500,100,-20,6;600,120,-10,6')
            int_name = self.get_parameter(f'intruder_name_{i}').value
            int_wp_string = self.get_parameter(f'intruder_waypoints_{i}',).value
            self.intruders_info_string[int_name] = separate_wp(int_wp_string)
            # Parse it to divide the waypoint strings wher % divide the waypoints ; the waypoint and , the variable
            self.intruders_info[int_name] = parse_multiple_wp_strings(int_wp_string)

        # Start the plot as teh Airplane trajectory:
        plt.ion()
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        plt.show(block=False)
        # Generate the color map for ht eplots of the intruders:
        self.color_map = generate_inturders_color_maps(self.intruders_info.keys())

        # QoS reliable to match with the c++ states:
        q_reliable = QoSProfile(depth=10)
        q_reliable.reliability = ReliabilityPolicy.RELIABLE
        q_reliable.durability  = DurabilityPolicy.VOLATILE

        # QoS for the trajectory complete flag topic of all the intruders
        q_events = q_reliable

        # Create the subscription to see if any of the intruders or the avoider are near the goal:
        # Avoider:
        self.avoider_traj_complete_sub_ = self.create_subscription(
            Bool,
            f"/{self.avoider_name}/traj_complete",
            lambda msg, name=self.avoider_name: self.traj_complete_callback(msg, name),
            q_events
        )
        # Intruders
        self.intruders_traj_complete_subs = []
        for name in self.intruders_info.keys():
            sub = self.create_subscription(
                Bool,
                f"/{name}/traj_complete",
                lambda msg, n=name: self.traj_complete_callback(msg, n),
                q_events
            )
            self.intruders_traj_complete_subs.append(sub)

        # Publish in case one of the USVs reach the goal:
        qos_2 = QoSProfile(depth=1)
        qos_2.reliability = ReliabilityPolicy.RELIABLE
        qos_2.durability = DurabilityPolicy.VOLATILE   
        self.complete_trajectory_pub_ = self.create_publisher(Bool, 'complete_encounter', qos_2)

        # Dictionary to save the running proccess:
        self.running_procs = {}

        # Subscribe to the sates of the intruders and the avoider:
        # Avoider
        self.own_airplane_states_sub_ = self.create_subscription(
            AirplaneKineticModelInfo,
            f"/{self.avoider_name}/states",
            self.avoider_states_callback,
            q_reliable
            )
        
        # Save the position information to plot them:
        self.avoider_states_north = []
        self.avoider_states_east = []
        self.avoider_states_alt = []

        # Information to save the trajectories 
        # Index to see what values the time choose:
        self.avoider_size = []
        # time:
        self.real_time_s = []
        self.sim_time_s = []
        # AVoider
        self.avoider_x_ft = []
        self.avoider_y_ft = []
        self.avoider_alt_ft = []
        self.avoider_vx_fts = []
        self.avoider_vy_fts = []
        self.avoider_vz_fts = []
        self.avoider_roll_rad = []
        self.avoider_pitch_rad = []
        self.avoider_yaw_rad = []

        # Indtruders:
        self.intruders_intruders_states_subs = []
        self.intuders_adsb_subs = []
        # Save the position states inforamtion of the intruders in dictionaries:
        self.intruder_states_north_ft = {}
        self.intruder_states_east_ft = {}
        self.intruder_states_up_ft = {}
        self.intruder_states_v_north = {}
        self.intruder_states_v_east = {}
        self.intruder_states_v_up = {}
        self.intruder_states_roll = {}
        self.intruder_states_fpa = {}
        self.intruder_states_course = {}
        self.intruder_states_roll_speed = {}
        self.intruder_states_velocity_a = {}
        # Save the adsb_inforamtion fo the intruders:
        self.intruder_adsb_north_ft = {}
        self.intruder_adsb_east_ft = {}
        self.intruder_adsb_up_ft = {}
        self.intruder_adsb_v_north = {}
        self.intruder_adsb_v_east = {}
        self.intruder_adsb_v_up = {}
        self.intruder_adsb_velocity_a = {}
        # Size of the intruders:
        self.intruder_size = {}
        if self.save_adsb_info:
            self.intruder_adsb_size = {}
        # SUbscribe to each intruder state:
        for name in self.intruders_info.keys():
            states_sub = self.create_subscription(
                AirplaneKineticModelInfo,
                f"/{name}/states",
                lambda msg, n=name: self.in_states_callback(msg,n),
                q_reliable
            )

            self.intruders_intruders_states_subs.append(states_sub)
            # Create empty lists in the dictionaries:
            self.intruder_size[name] = []
            self.intruder_states_north_ft[name] = []
            self.intruder_states_east_ft[name] = []
            self.intruder_states_up_ft[name] = []
            self.intruder_states_v_north[name] = []
            self.intruder_states_v_east[name] = []
            self.intruder_states_v_up[name] = []
            self.intruder_states_roll[name] = []
            self.intruder_states_fpa[name] = []
            self.intruder_states_course[name] = []
            self.intruder_states_roll_speed[name] = []
            self.intruder_states_velocity_a[name] = []

            # Save in case you want ot save the adsb out:
            if self.save_adsb_info:
                self.intruder_adsb_size[name] = []

            # Start a subscriber tot eh adsb_out data:
            adsb_sub = self.create_subscription(
                AdsbData,
                f"/{name}/adsb_out",
                lambda msg, n=name: self.in_adsb_callback(msg,n),
                q_reliable
            )
            self.intuders_adsb_subs.append(adsb_sub)

            # Create an empty list for each intruder:
            self.intruder_adsb_north_ft[name] = []
            self.intruder_adsb_east_ft[name] = []
            self.intruder_adsb_up_ft[name] = []
            self.intruder_adsb_v_north[name] = []
            self.intruder_adsb_v_east[name] = []
            self.intruder_adsb_v_up[name] = []
            self.intruder_adsb_velocity_a[name] = []
        
        # Generate the Gazebo service to  dynamically change the position of the UAVs:
        self.move_the_airplane_client = self.create_client(SetEntityState, '/gazebo/set_entity_state') # Remember to add the gazebo_ros_state plugin to the world to make it work

        # Topic to close the simulation where all the sets are completed:
        self.stop_pub = self.create_publisher(Bool,'/stop_simulation',1)

        # Gnerate a publisher to publish the intruders information to the DAA UAV follower:
        self.intruders_info_pub_ = self.create_publisher(AvoidanceStates, "obstacles_states",qos_2)
        # Add a timer so the system send the intruder information in an specified frequency:
        self.timer = self.create_timer(1/2,self.adsb_callback)

        # Add a timer to save data of the ownhip adn the intruders:
        self.timer_save = self.create_timer(0.1,self.data_callback)

        # Subscriber of the YOLO detection system:
        self.airplane_recog_sub = self.create_subscription(
            Yolov11Inference360,
            f"/{self.avoider_name}/YOLOv11_inference",
            self.score_airplane_callback,
            10
        )
            
        # Restart the variables before strating the simulation:
        self.restart_variables()

        ## Porcess to close the nodes when is closed by ctrl+c and there is an error:
        atexit.register(self._cleanup, reason="atexit")
        try:
            # works on Humble+: call on rclpy shutdown as well
            import rclpy
            rclpy.get_default_context().on_shutdown(lambda: self._cleanup("rclpy.on_shutdown"))
        except Exception:
            pass
        signal.signal(signal.SIGINT,  self._sig_handler)   # Ctrl+C
        signal.signal(signal.SIGTERM, self._sig_handler)   # kill/terminate

    
    # Function to tell all topic listeners to stop, when the simualiton is stopped:
    def _sig_handler(self, signum, frame):
        self.get_logger().warn(f"Signal {signum} received -> stopping followers")
        with contextlib.suppress(Exception):
            self.stop_pub.publish(Bool(data=True))
        self.stop_wait_the_followers(grace1=1.5, grace2=1.5)

    
    # Clean-up function to clean all the reamining nodes when it is closed:
    def _cleanup(self, reason=""):
        try:
            self.get_logger().info(f"Cleanup invoked ({reason})")
        except Exception:
            pass
        with contextlib.suppress(Exception):
            self.stop_pub.publish(Bool(data=True))
        # Try graceful, then hard
        self.stop_wait_the_followers(grace1=1.5, grace2=1.5)
        with contextlib.suppress(Exception):
            import matplotlib.pyplot as plt
            plt.close('all')

    # Save the scores saved depending on the camera that detects it
    def score_airplane_callback(self, msg):
        return

    # Restart all the private values used inside the executable;
    def restart_variables(self):
        # Varible to obaitn the initial time where the trajecotry start:
        self.initial_real_time = None
        self.initial_sim_time = None
        # List with all the intruders:
        self.intruders_list_ready = set(self.intruders_info.keys())
        # Flag to see if the avoider and the intruders are ready
        self.intruders_ready = False
        self.avoider_ready = False
        # Flag to see if any UAV compelte the trejctory:
        self.traj_complete = False
        # Restart the dictioanries with the actual states data of the UAVs:
        # Time:
        self.real_time = 0
        self.sim_time = 0
        # Avoider:
        self.avoider_x = 0
        self.avoider_y = 0
        self.avoider_alt = 0
        # PLotting:
        self.avoider_states_north = []
        self.avoider_states_east = []
        self.avoider_states_alt = []
        # Size of the lists:
        self.avoider_size = []
        # time:
        self.real_time_s = []
        self.sim_time_s = []
        # Save information:
        self.avoider_x_ft = []
        self.avoider_y_ft = []
        self.avoider_alt_ft = []
        self.avoider_vx_fts = []
        self.avoider_vy_fts = []
        self.avoider_vz_fts = []
        self.avoider_roll_rad = []
        self.avoider_pitch_rad = []
        self.avoider_yaw_rad = []

        
        # Intruders:
        # Plotting
        hist_intruders = [
            self.intruder_size,
            self.intruder_states_north_ft,
            self.intruder_states_east_ft,
            self.intruder_states_up_ft,
            self.intruder_states_v_north,
            self.intruder_states_v_east,
            self.intruder_states_v_up,
            self.intruder_states_roll,
            self.intruder_states_fpa,
            self.intruder_states_course,
            self.intruder_states_roll_speed,
            self.intruder_states_velocity_a,
            self.intruder_adsb_north_ft,
            self.intruder_adsb_east_ft,
            self.intruder_adsb_up_ft,
            self.intruder_adsb_v_north,
            self.intruder_adsb_v_east,
            self.intruder_adsb_v_up,
            self.intruder_adsb_velocity_a,
        ]

        self.running_procs = {}

        for list_hist in hist_intruders:
            for k in list_hist:
                list_hist[k].clear()
                if self.save_adsb_info:
                    self.intruder_adsb_size[k].clear()
        


    # Ploting system to restart the ploting and plot the new trajectories every timie it completes one wncounter set:
    def setup_plot(self,avoider_traj, num_traectory):
        for _ in range(5):
            rclpy.spin_once(self, timeout_sec=0.1)
        
        # Call the axis fo the plot
        self.ax.clear()
        self.ax.set_xlabel('East (ft)')
        self.ax.set_ylabel('North (ft)')
        self.ax.set_zlabel('Altitude (ft)')
        self.ax.set_title('UAV Waypoint follower Monitoring')
        # Gnerate three lists to save the limits of the graph:
        east_points, north_points, alt_points = [], [], []

        # Plot the Avoider Trajectory:
        avoider_e_ft = avoider_traj[:,1] / 0.3048
        avoider_n_ft = avoider_traj[:,0] / 0.3048
        avoider_alt_ft = avoider_traj[:,2] / 0.3048
        # Append to the points:
        east_points.append(avoider_e_ft)
        north_points.append(avoider_n_ft)
        alt_points.append(avoider_alt_ft)
        # Plot the avoider waypoitns:
        self.ax.plot(avoider_e_ft, avoider_n_ft, avoider_alt_ft, 'ro--', alpha = 0.9, label='Ownship Waypoints')
        # Generate the 2D line for the avoider:
        (self.line_states_avoider,) = self.ax.plot([],[],[], 'b', label = 'Avoider path')

        # Plot teh Intruders Trajectory:
        for name, segments in self.intruders_info.items():
            # Open the respective waypoint information
            int_trajectory = segments[num_traectory]
            int_e_ft = int_trajectory[:, 1] / 0.3048
            int_n_ft = int_trajectory[:, 0] / 0.3048
            int_alt_ft = int_trajectory[:, 2] / 0.3048
            # Save it in the poitns lists;
            east_points.append(int_e_ft)
            north_points.append(int_n_ft)
            alt_points.append(int_alt_ft)
            self.ax.plot(int_e_ft,int_n_ft,int_alt_ft, 
                        color = self.color_map[name]['color'],
                        linestyle=self.color_map[name]['line_style'],
                        alpha = 0.9,
                        label=f"Intruder {name}"
                        )
        # Generate the dynamics plot for the  intruders:
        for name in self.intruders_info.keys():
            (line,) = self.ax.plot([],[],[], 'm')
            self.intruder_lines[name] = line

        # Plot the legends:
        self.ax.legend()

        # Find teh limits of the Waypoints to do an axis equal:
        e_all = np.concatenate(east_points)
        n_all = np.concatenate(north_points)
        alt_all = np.concatenate(alt_points)
        max_range = max( np.ptp(e_all),np.ptp(n_all),np.ptp(alt_all)) / 2.0
        mid_x = np.mean(e_all)
        mid_y = np.mean(n_all)
        mid_z = np.mean(alt_all)
        self.ax.set_xlim(mid_x - max_range, mid_x + max_range)
        self.ax.set_ylim(mid_y - max_range, mid_y + max_range)
        self.ax.set_zlim(mid_z - max_range, mid_z + max_range)
        plt.show(block=False)

    
    # Detect when the Airplane its near the goal position:
    def traj_complete_callback(self,msg, name):
        if msg.data and not getattr(self, "traj_complete", False):
            self.traj_complete = True
            self.complete_trajectory_pub_.publish(Bool(data=True))
            self.get_logger().info(f"{name} completed the trajectory; signaling encounter completion")

    
    def avoider_states_callback(self, msg):
        if not hasattr(self, "line_states_avoider"):
            return
        if msg.velocity_a > 0:

            # Get the times
            self.real_time = msg.real_time
            self.sim_time = msg.sim_time
            self.avoider_ready = True

            # Position in ft
            self.avoider_y = msg.north/0.3048
            self.avoider_x = msg.east/0.3048
            self.avoider_alt = msg.up/0.3048

            # Append states
            self.avoider_states_north.append(self.avoider_y)
            self.avoider_states_east.append(self.avoider_x)
            self.avoider_states_alt.append(self.avoider_alt)

            # Save the state information:
            self.avoider_x_ft.append(msg.east/0.3048)
            self.avoider_y_ft.append(msg.north/0.3048)
            self.avoider_alt_ft.append(msg.up/0.3048)
            self.avoider_roll_rad.append(msg.roll)
            self.avoider_pitch_rad.append(-msg.fpa)
            self.avoider_yaw_rad.append(np.pi/2-msg.course)
            self.avoider_vx_fts.append(msg.v_east)
            self.avoider_vy_fts.append(msg.v_north)
            self.avoider_vz_fts.append(msg.v_up)

            # Plot the trajecotry
            self.line_states_avoider.set_data(
            self.avoider_states_east,
            self.avoider_states_north
            )
            self.line_states_avoider.set_3d_properties(
                self.avoider_states_alt
            )

            # Just redraw; no relim/autoscale for 3D
            plt.draw()
            plt.pause(0.001)

    

    # Intruders states callback and asve the inoramtion in a dictionary of lists to plot them;
    def in_states_callback(self, msg, name):
        # If this intruder doesn't have a line, ignore
        if name not in self.intruder_lines:
            return

        if msg.velocity_a > 0:
            # Analyze when all the intruders are ready to use the timer:
            if name in self.intruders_list_ready:
                self.intruders_list_ready.discard(name)
                if not self.intruders_list_ready:
                    self.intruders_ready = True

            # Convert Gazebo values to feet (do it once)
            north_ft = msg.north / 0.3048
            east_ft  = msg.east  / 0.3048
            alt_ft   = msg.up    / 0.3048

            # Save positions in dictionaries of lists
            self.intruder_states_north_ft[name].append(north_ft)
            self.intruder_states_east_ft[name].append(east_ft)
            self.intruder_states_up_ft[name].append(alt_ft)

            # Plot the intruder trajecotry:
            line_of_intruder = self.intruder_lines[name]
            line_of_intruder.set_data(
                self.intruder_states_east_ft[name],
                self.intruder_states_north_ft[name]
            )
            line_of_intruder.set_3d_properties(
                self.intruder_states_up_ft[name]
            )

            plt.draw()
            plt.pause(0.001)

            # Save the inforamtion of the inturders in to published in the timer callback made by the AvidanceStates:
            # Orientation
            self.intruder_states_course[name].append(msg.course)
            self.intruder_states_fpa[name].append(msg.fpa)
            self.intruder_states_roll[name].append(msg.roll)
            # Lienar Velocity:
            self.intruder_states_v_north[name].append(msg.v_north/0.3048)
            self.intruder_states_v_east[name].append(msg.v_east/0.3048)
            self.intruder_states_v_up[name].append(msg.v_up/0.3048)
            # Overall velcoity and roll speed
            self.intruder_states_velocity_a[name].append(msg.velocity_a)
            self.intruder_states_roll_speed[name].append(msg.roll_speed)

    # ADS-B states callback and asve the inoramtion in a dictionary of lists to plot them;
    def in_adsb_callback(self,msg, name):
        if name not in self.intruder_lines:
            return
        
        # Calcaulte the velocity norm:
        v_north = msg.v_north
        v_east = msg.v_east
        v_up = msg.v_up
        velocity_a = np.sqrt(v_north*v_north+v_east*v_east+v_up*v_up)
        if velocity_a > 0:
            # Analyze when all the intruderas are ready to use the timer:
            if name in self.intruders_list_ready:
                self.intruders_list_ready.discard(name)
                if not self.intruders_list_ready:
                    self.intruders_ready = True
            # Change the velues of Gazbeo to feets:
            north_ft = msg.north/0.3048
            east_ft = msg.east/0.3048
            alt_ft = msg.up/0.3048
            # PLot it in the Python figure:
            self.intruder_adsb_north_ft[name].append(north_ft)
            self.intruder_adsb_east_ft[name].append(east_ft)
            self.intruder_adsb_up_ft[name].append(alt_ft)
            # Save the inforamtion of the inturders in to published in the timer callback made by the AvidanceStates:
            self.intruder_adsb_v_north[name].append(v_north)
            self.intruder_adsb_v_east[name].append(v_east)
            self.intruder_adsb_v_up[name].append(v_up)
            # Overall velcoity and roll speed
            self.intruder_adsb_velocity_a[name].append(velocity_a)

    
    # Define a call Trigger services:
    def call_trigger(self, service_name: str, wait_timeout: float = 20.0) -> bool:
        client = self.create_client(Trigger, service_name)
        if not client.wait_for_service(timeout_sec=wait_timeout):
            self.get_logger().warn(f"[start] Service not available: {service_name}")
            return False
        req = Trigger.Request()
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=wait_timeout)
        if not future.done() or future.result() is None:
            self.get_logger().warn(f"[start] No response from: {service_name}")
            return False
        resp = future.result()
        if resp.success:
            self.get_logger().info(f"[start] Triggered: {service_name} ({resp.message})")
            return True
        self.get_logger().warn(f"[start] Failed: {service_name} ({resp.message})")
        return False


     # Funciton to spawn the airplanes in a position and with an orientation:
    def call_service_change_position(self, client, service_name: str, entity_name: str, east: float, north: float, alt: float, roll: float, fpa: float, course: float) -> bool:
        # Set the model name and the posiiton
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



    # Define the timer that is used ti publish the intruders inforamation to DAA:
    def adsb_callback(self):
        # Only start the timer if the intruder is ready:
        if not (self.avoider_ready and self.intruders_ready):
            return
        
        # Genrate a list of the name of the intruders:
        msg = AvoidanceStates()
        for name in self.intruders_info.keys():
            # Guard in case a list is still empty
            if not self.intruder_adsb_north_ft[name]:
                continue

            intr_states = AirplaneKineticModelInfo()
            # Position 
            intr_states.north = self.intruder_adsb_north_ft[name][-1] * 0.3048
            intr_states.east  = self.intruder_adsb_east_ft[name][-1]  * 0.3048
            intr_states.up    = self.intruder_adsb_up_ft[name][-1]    * 0.3048
            # Linear velocity
            intr_states.v_north = self.intruder_adsb_v_north[name][-1]
            intr_states.v_east  = self.intruder_adsb_v_east[name][-1]
            intr_states.v_up    = self.intruder_adsb_v_up[name][-1]
            # Orientation
            intr_states.roll   = float(0.1)
            intr_states.fpa    = float(0.1)
            intr_states.course = float(0.1)
            # Others
            intr_states.roll_speed = float(0.1)
            intr_states.velocity_a = self.intruder_adsb_velocity_a[name][-1]

            msg.intruder_states.append(intr_states)
            msg.obstacles_id.append(name)

        self.intruders_info_pub_.publish(msg)

    
    # Callack function to save the informaion of ht airplanes:
    def data_callback(self):
        # Only start the timer if the intruder is ready:
        if not (self.avoider_ready and self.intruders_ready):
            return
        
        # Require at least one avoider sample
        if len(self.avoider_x_ft) == 0:
            return

        # Require at least one state sample for every intruder
        for name in self.intruders_info.keys():
            if len(self.intruder_states_north_ft[name]) == 0:
                return
        
        # Initialize the reference times once (when we start logging)
        if self.initial_real_time is None or self.initial_sim_time is None:
            self.initial_real_time = self.real_time
            self.initial_sim_time = self.sim_time
        
        # Save the information of teh ownship:
        self.real_time_s.append(self.real_time-self.initial_real_time)
        self.sim_time_s.append(self.sim_time-self.initial_sim_time)

        # Save the indices of the avoider:
        if len(self.avoider_x_ft) >=1:
            self.avoider_size.append(len(self.avoider_x_ft))

        # Save the indices of the intruders:
        # For the case the ADAS-B information is not requires:
        if self.save_adsb_info:
            for name in self.intruders_info.keys():
                if len(self.intruder_states_north_ft[name]) >= 1:
                    self.intruder_size[name].append(len(self.intruder_states_north_ft[name]))
                    self.intruder_adsb_size[name].append(len(self.intruder_adsb_north_ft[name]))
        else:
            for name in self.intruders_info.keys():
                if len(self.intruder_states_north_ft[name]) >= 1:
                    self.intruder_size[name].append(len(self.intruder_states_north_ft[name]))
            
    

    # Function to spawn the airplane at the initial position by using the first waypoints:
    def spawn_the_UAV_using_waypoints(self, name, trajectory):
        # Define the teo points:
        pos0 = trajectory[0,:]
        pos1 = trajectory[1,:]
        # Calcualte the required deltas:
        delta_pos = pos1 - pos0

        # Define the angles of orientation of the UAV:
        # Fligth path angle:
        fpa = np.arctan2(delta_pos[2], np.hypot(delta_pos[0], delta_pos[1]))
        # Course angle:
        course = np.arctan2(delta_pos[1], delta_pos[0]) 
        
        # Start the spawning:
        if not self.call_service_change_position(self.move_the_airplane_client, '/gazebo/set_entity_state',name, pos0[1],pos0[0],pos0[2],0.0,fpa,course):
                        return
        time.sleep(5)

    
    # Start the movement pluggins of the avoider and the intruders:
    def start_airplane_movment(self, idx):
        # First laucnh the avoider:
        own_args = [
            "ros2", "run", "airplane_gazebo_plugin", "geometric_wp_detect_and_avoidance", self.avoider_name, self.avoider_wp_string[idx]
        ]
        self.get_logger().info(f"Starting the avoider follower")
        self.running_procs["__ownship__"] = subprocess.Popen(
            own_args,
            start_new_session=True,
        )

        # Then add hte intruders:
        for name, seg_list in self.intruders_info_string.items():
            int_args = [
                "ros2", "run", "airplane_gazebo_plugin", "airplane_waypoint_follower", name, seg_list[idx]
            ]
            self.running_procs[name] = subprocess.Popen(
                int_args,
                start_new_session=True,
            )
    

    # Wait until any of the airplanes get to thir respective goals:
    def wait_until_airplanes_complete(self, timeout_s: float = 60.0):
        # Start the camera detection system:
        if not self.call_trigger(f"/{self.avoider_name}_start_detection"):
                return

        # Start the airplane movement triggering:
        # Avoider:
        if not self.call_trigger(f"/{self.avoider_name}/trigger_service"):
                return
        # Intruders:
        for name in self.intruders_info.keys():
            if not self.call_trigger(f"/{name}/trigger_service"):
                return
        # Intruder adsb_start:
        for name in self.intruders_info.keys():
            if not self.call_trigger(f"/{name}/adsb_start_service"):
                return

        # Starting time:
        start = time.time()

        while True:
            if getattr(self, "traj_complete", False):
                return "traj_complete"

            any_alive = any(p.poll() is None for p in self.running_procs.values())
            if not any_alive:
                return "all_exited"

            # condition (c): timeout
            if (time.time() - start) > timeout_s:
                return "timeout"

            rclpy.spin_once(self, timeout_sec=0.1)

    
    # Keep the system running while the followers are closing:
    def stop_wait_the_followers(self, grace1: float = 6.0, grace2: float = 5.0):
        if not self.running_procs:
            return
        
        # Give them time to close by themselves:
        t_end = time.time() + grace1
        while time.time() < t_end:
            if all(p.poll() is not None for p in self.running_procs.values()):
                break
            rclpy.spin_once(self, timeout_sec=0.1)

        # Stop any remainning process that is still open;
        still_alive = [p for p in self.running_procs.values() if p.poll() is None]
        for p in still_alive:
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGINT)
            except Exception:
                p.terminate()
        
        # Stop the camera deteciton system:
        if not self.call_trigger(f"/{self.avoider_name}_stop_detection"):
                return


        # Intruder adsb_stop:
        for name in self.intruders_info.keys():
            if not self.call_trigger(f"/{name}/adsb_stop_service"):
                return
            
        # Wait a bit more of time
        t_end = time.time() + grace2
        while time.time() < t_end:
            if all(p.poll() is not None for p in self.running_procs.values()):
                break
            rclpy.spin_once(self, timeout_sec=0.1)
        
        # Hard kill the stragglers:
        for name, p in list(self.running_procs.items()):
            if p.poll() is None:
                try:
                    os.killpg(os.getpgid(p.pid), signal.SIGKILL)
                except Exception:
                    p.kill()
            self.running_procs.pop(name, None)



    # Function to save the ownship and intruder information.
    def save_information(self,Number_of_Trajectory):
        # Create the folder of the fligth:
        fligth_folder = os.path.join(self.save_dir,f"Encounter_{Number_of_Trajectory}")
        os.makedirs(fligth_folder)

        # Create a folder for ht airplanes:
        folder_airplane_states = os.path.join(fligth_folder, "states_data")
        os.makedirs(folder_airplane_states)

        # Generate the CSV file for the ownship:
        ownship_states_file = os.path.join(folder_airplane_states, "ownship_states.csv")
        with open(ownship_states_file, 'w') as file:
            file.write("Real_time_s,Simulation_time_s,x_ft,y_ft,alt_ft,vx_fps,vy_fps,vz_fps,roll_rad,pitch_rad,yaw_rad\n")
            for i in range(len(self.real_time_s)):
                # Get the index value:
                idx_avoider = self.avoider_size[i]-1
                # Wirite the line
                file.write(f"{self.real_time_s[i]},{self.sim_time_s[i]},{self.avoider_x_ft[idx_avoider]},{self.avoider_y_ft[idx_avoider]},{self.avoider_alt_ft[idx_avoider]},{self.avoider_vx_fts[idx_avoider]},{self.avoider_vy_fts[idx_avoider]},{self.avoider_vz_fts[idx_avoider]},{self.avoider_roll_rad[idx_avoider]},{self.avoider_pitch_rad[idx_avoider]},{self.avoider_yaw_rad[idx_avoider]}\n")
        

        # For the intruders: 
        for name in self.intruders_info.keys():
            intruder_state_file = os.path.join(folder_airplane_states, f"{name}_states.csv")
            with open(intruder_state_file, 'w') as file:
                if self.save_adsb_info:
                    file.write("Real_time_s,Simulation_time_s,x_ft,y_ft,alt_ft,vx_fps,vy_fps,vz_fps,roll_rad,pitch_rad,yaw_rad,adsb_x_ft,adsb_y_ft,adsb_alt_ft\n")
                else:
                    file.write("Real_time_s,Simulation_time_s,x_ft,y_ft,alt_ft,vx_fps,vy_fps,vz_fps,roll_rad,pitch_rad,yaw_rad\n")

                for i in range(len(self.real_time_s)):
                    if self.save_adsb_info:
                        # Get the index of the information:
                        idx_intruder = self.intruder_size[name][i]-1
                        idx_intruder_adsb = self.intruder_adsb_size[name][i]-1
                        # Wirite the line
                        file.write(f"{self.real_time_s[i]},{self.sim_time_s[i]},{self.intruder_states_east_ft[name][idx_intruder]},{self.intruder_states_north_ft[name][idx_intruder]},{self.intruder_states_up_ft[name][idx_intruder]},{self.intruder_states_v_east[name][idx_intruder]},{self.intruder_states_v_north[name][idx_intruder]},{self.intruder_states_v_up[name][idx_intruder]},{self.intruder_states_roll[name][idx_intruder]},{-1*self.intruder_states_fpa[name][idx_intruder]},{np.pi/2-self.intruder_states_course[name][idx_intruder]},{self.intruder_adsb_east_ft[name][idx_intruder_adsb]},{self.intruder_adsb_north_ft[name][idx_intruder_adsb]},{self.intruder_adsb_up_ft[name][idx_intruder_adsb]}\n")
                    else:
                        # Get the index of the information:
                        idx_intruder = self.intruder_size[name][i]-1
                        # Wirite the line
                        file.write(f"{self.real_time_s[i]},{self.sim_time_s[i]},{self.intruder_states_east_ft[name][idx_intruder]},{self.intruder_states_north_ft[name][idx_intruder]},{self.intruder_states_up_ft[name][idx_intruder]},{self.intruder_states_v_east[name][idx_intruder]},{self.intruder_states_v_north[name][idx_intruder]},{self.intruder_states_v_up[name][idx_intruder]},{self.intruder_states_roll[name][idx_intruder]},{-1*self.intruder_states_fpa[name][idx_intruder]},{np.pi/2-self.intruder_states_course[name][idx_intruder]}\n")
 
        

        

    # Start witht he overall logic of the DAA by spawning teh UAVs, Start the Waypoint follower and the avoider DAA executables:
    def daa_simulation_setup(self):
        # Iterate in all the encoutners starting each daa case:
        for i, avoider_traj in enumerate(self.avoider_wp_info):
            # Restart the settings and the list saved for 
            self.restart_variables()
            time.sleep(5)

            # Start the plot setup 
            self.setup_plot(avoider_traj, i)
            # Draw the plot:
            plt.draw()
            plt.pause(0.5)

            # Spawn the airplanes in the initial position:
            # The avoider:
            self.spawn_the_UAV_using_waypoints(self.avoider_name,avoider_traj)
            # Spawn the intruders;
            for name, segments in self.intruders_info.items():
                self.spawn_the_UAV_using_waypoints(name,segments[i])

            # Run the executable to move the ownship and hte intruder:
            # Strat the trajecotry following nodes:
            self.start_airplane_movment(i)
            # Keep it running and trigger the movemnt:
            self.wait_until_airplanes_complete(timeout_s=500.0)
            # Clean up followers before moving to next encounter
            self.stop_wait_the_followers()
            self.save_information(i)
            time.sleep(5.0)
        
        # Stop teh simulation when the process finish
        msg = Bool()
        msg.data = True
        self.stop_pub.publish(msg)


            
def main(args=None):
    rclpy.init(args=args)
    node = DAASimulation()
    try:
         node.daa_simulation_setup()
    except KeyboardInterrupt:
        node.get_logger().warn("Interrupted by user (Ctrl+C). Cleaning up...")
        node._cleanup("KeyboardInterrupt")
    except Exception as e:
        node.get_logger().error(f"Unhandled exception: {e}. Cleaning up...")
        node._cleanup("exception")
        raise
    finally:
        # final safety net (idempotent)
        node._cleanup("finally")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()