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
from airplane_gazebo_plugin.msg import AirplaneKineticModelInfo, AvoidanceStates
from custom_msgs.msg import Yolov11Inference360
# Service to spawn the airplane in a position 
from gazebo_msgs.srv import SetEntityState

# Library to start process on the command window:
import subprocess


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


# Function to separate the multiple waypoints in a list of strings:
def separate_wp(mult_wp_string):
    waypoints_strings = []
    for wps in mult_wp_string.split('%'):
        wps = wps.strip()
        waypoints_strings.append(wps)
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

        # Plot Characteristic
        self.intruder_lines = {}

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

        # QoS for the events and flags during the simulation:
        q_states = QoSProfile(depth=1)
        q_states .reliability = ReliabilityPolicy.RELIABLE
        q_states .durability  = DurabilityPolicy.VOLATILE

        # QoS for the trajectory complete flag topic of all the intruders
        q_events = QoSProfile(depth=10)
        q_events.reliability = ReliabilityPolicy.RELIABLE
        q_events.durability  = DurabilityPolicy.VOLATILE

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
        # Indtruders:
        self.intruders_intruders_states_subs = []
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
        for name in self.intruders_info.keys():
            states_sub = self.create_subscription(
                AirplaneKineticModelInfo,
                f"/{name}/states",
                lambda msg, n=name: self.in_states_callback(msg,n),
                q_reliable
            )
            self.intruders_intruders_states_subs.append(states_sub)
            # Create empty lists in the dictionaries:
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
        
        # Generate the Gazebo service to  dynamically change the position of the UAVs:
        self.move_the_airplane_client = self.create_client(SetEntityState, '/gazebo/set_entity_state') # Remember to add the gazebo_ros_state plugin to the world to make it work

        # Topic to close the simulation where all the sets are completed:
        self.stop_pub = self.create_publisher(Bool,'/stop_simulation',1)

        # Gnerate a publisher to publish the intruders information to the DAA UAV follower:
        self.intruders_info_pub_ = self.create_publisher(AvoidanceStates, "obstacles_states",qos_2)
        # Add a timer so the system send the intruder information in an specified frequency:
        self.timer = self.create_timer(1/2,self.timer_callback)
            
        # Restart the variables before strating the simulation:
        self.restart_variables()
    

    # Restart all the private values used inside the executable;
    def restart_variables(self):
        # List with all the intruders:
        self.intruders_list_ready = set(self.intruders_info.keys())
        # Flag to see if the avoider and the intruders are ready
        self.intruders_ready = False
        self.avoider_ready = False
        # Flag to see if any UAV compelte the trejctory:
        self.traj_complete = False
        # Restart the dictioanries with the actual states data of the UAVs:
        # Avoider:
        self.avoider_x = 0
        self.avoider_y = 0
        self.avoider_alt = 0
        # PLotting:
        self.avoider_states_north = []
        self.avoider_states_east = []
        self.avoider_states_alt = []
        # Intruders:
        # Plotting
        hist_intruders = [
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
        ]

        for list_hist in hist_intruders:
            for k in list_hist:
                list_hist[k].clear()


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

    
    # States callback to save it in a list and plot them:
    def avoider_states_callback(self,msg):
        if not hasattr(self, "line_states_avoider"):
            return
        if msg.velocity_a > 0:
            # Tell the executbale that the avoider is ready:
            self.avoider_ready = True
            # Position:
            self.avoider_y = msg.north/0.3048
            self.avoider_x = msg.east/0.3048
            self.avoider_alt = msg.up/0.3048
            # Plot the path in Python:
            self.avoider_states_north.append(self.avoider_y)
            self.avoider_states_east.append(self.avoider_x)
            self.avoider_states_alt.append(self.avoider_alt)
            self.line_states_avoider.set_data(self.avoider_states_east, self.avoider_states_north)
            self.line_states_avoider.set_3d_properties(self.avoider_states_alt)
            self.ax.relim()
            self.ax.autoscale_view()
            plt.draw()
            plt.pause(0.001)
    

    # Intruders states callback and asve the inoramtion in a dictionary of lists to plot them;
    def in_states_callback(self,msg, name):
        if name not in self.intruder_lines:
            return
        if msg.velocity_a > 0:
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
            self.intruder_states_north_ft[name].append(north_ft)
            self.intruder_states_east_ft[name].append(east_ft)
            self.intruder_states_up_ft[name].append(alt_ft)
            #Select the line of the intruder and plot the information:
            line_of_intruder = self.intruder_lines[name]
            line_of_intruder.set_data(self.intruder_states_east_ft[name], self.intruder_states_north_ft[name])
            line_of_intruder.set_3d_properties(self.intruder_states_up_ft[name])
            self.ax.relim()
            self.ax.autoscale_view()
            plt.draw()
            plt.pause(0.001)

            # Save the inforamtion of the inturders in to published in the timer callback made by the AvidanceStates:
            # Orientation
            self.intruder_states_course[name].append(msg.course)
            self.intruder_states_fpa[name].append(msg.fpa)
            self.intruder_states_roll[name].append(msg.roll)
            # Lienar Velocity:
            self.intruder_states_v_north[name].append(msg.v_north)
            self.intruder_states_v_east[name].append(msg.v_east)
            self.intruder_states_v_up[name].append(msg.v_up)
            # Overall velcoity and roll speed
            self.intruder_states_velocity_a[name].append(msg.velocity_a)
            self.intruder_states_roll_speed[name].append(msg.roll_speed)


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
    def timer_callback(self):
        # Only start the timer if the intruder is ready:
        if not (self.avoider_ready and self.intruders_ready):
            return

        # Genrate a list of the name of the intruders:
        msg = AvoidanceStates()
        for name in self.intruders_info.keys():
            # Guard in case a list is still empty
            if not self.intruder_states_north_ft[name]:
                continue

            intr_states = AirplaneKineticModelInfo()
            # Position 
            intr_states.north = self.intruder_states_north_ft[name][-1] * 0.3048
            intr_states.east  = self.intruder_states_east_ft[name][-1]  * 0.3048
            intr_states.up    = self.intruder_states_up_ft[name][-1]    * 0.3048
            # Linear velocity
            intr_states.v_north = self.intruder_states_v_north[name][-1]
            intr_states.v_east  = self.intruder_states_v_east[name][-1]
            intr_states.v_up    = self.intruder_states_v_up[name][-1]
            # Orientation
            intr_states.roll   = self.intruder_states_roll[name][-1]
            intr_states.fpa    = self.intruder_states_fpa[name][-1]
            intr_states.course = self.intruder_states_course[name][-1]
            # Others
            intr_states.roll_speed = self.intruder_states_roll_speed[name][-1]
            intr_states.velocity_a = self.intruder_states_velocity_a[name][-1]

            msg.intruder_states.append(intr_states)
            msg.obstacles_id.append(name)

        self.intruders_info_pub_.publish(msg)
    

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

        
    # Start witht he overall logic of the DAA by spawning teh UAVs, Start the Waypoint follower and the avoider DAA executables:
    def daa_simulation_setup(self):
        # Iterate in all the encoutners starting each daa case:
        for i, avoider_traj in enumerate(self.avoider_wp_info):
            # Restart the settings and the list saved for 
            self.restart_variables()
            time.sleep(10)

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
            # Try the code until here:
            time.sleep(60)


            
def main(args=None):
    rclpy.init(args=args)
    node = DAASimulation()
    try:
        node.daa_simulation_setup()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()