# Import ROS2 Libraries
from tkinter import SEL
import rclpy
from rclpy.node import Node

# Import ROS2 odometry msg
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32

# Import math libraries
import math

# Import the connection library
import os

# Get the performance metrics:
import psutil

class SavePerformanceMetrics(Node):
    def __init__(self):
        # Start the node:
        super().__init__('save_performance_metrics')

        # Declare parameters:
        self.declare_parameter('output_folder', 'default_output_folder')
        self.declare_parameter('frecuency', 10)

        # Get parameters:
        self.output_folder = self.get_parameter('output_folder').value
        self.frequency = self.get_parameter('frecuency').value

        # Print the parameters to see how they are working:
        self.get_logger().info(f"Output Folder: {self.output_folder}")
        self.get_logger().info(f"Output Folder: {self.frequency}")

        # Define a variable that checks if the simulation started:
        self.simulation_started = False

        # Add a counter to start when the simulation started and take out the delay time values:
        self.initial_real_time = 0
        self.count_real = 0
        self.initial_sim_time = 0
        self.count_sim = 0

        # Generate the lists of the elements that are going to be saved:
        self.REAL_TIME_s = []
        self.SIM_TIME_s = []
        self.RTF = []
        self.CPU_Usage = []
        self.RAM_Usage = []

        # Generate the subscribers:
        self.real_time_sub = self.create_subscription(Float32, '/real_time', self.real_time_callback, 10)
        self.sim_time_sub = self.create_subscription(Float32, '/sim_time', self.sim_time_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/airplane_1/odom', self.odom_callback, 10)

        # Generate a timer to save the information:
        self.timer = self.create_timer(1/self.frequency,self.timer_callback)





    def odom_callback(self, msg):
        vel_x = msg.twist.twist.linear.y
        vel_y = msg.twist.twist.linear.x
        vel_z = -msg.twist.twist.linear.z
        vel = math.sqrt(vel_x**2 + vel_y**2 + vel_z**2)
        # Find if the simulation started
        if vel > 0.5:
            self.simulation_started = True
    
    def real_time_callback(self, msg):
        if self.simulation_started and self.count_real == 0:
            self.initial_real_time = msg.data
            self.count_real = 1

        if self.simulation_started and self.count_real == 1:
            self.time_s=msg.data - self.initial_real_time

    def sim_time_callback(self, msg):
        if self.simulation_started and self.count_sim == 0:
            self.initial_sim_time = msg.data
            self.count_sim = 1

        if self.simulation_started and self.count_sim == 1:
            self.time_sim_s=msg.data - self.initial_sim_time
    
    def timer_callback(self):
        if self.simulation_started:
            self.REAL_TIME_s.append(self.time_s)
            self.SIM_TIME_s.append(self.time_sim_s)
            self.RTF.append(self.time_sim_s/self.time_s)
            self.CPU_Usage.append(max(psutil.cpu_percent(0.1),0.0))
            self.RAM_Usage.append((psutil.virtual_memory().used)/(10**9))
    
    def save_performance_data(self):
        file_name = 'performance_metrics.csv'
        file_path = os.path.join(self.output_folder, file_name)

        #Save the information:
        with open(file_path,'w') as file:
            file.write("Real_time_s,Simulation_time_s,Real_time_factor,%CPU_used,GB_of_RAM\n")
            for i in range(len(self.REAL_TIME_s)):
                file.write(f"{self.REAL_TIME_s[i]},{self.SIM_TIME_s[i]},{self.RTF[i]},{self.CPU_Usage[i]},{self.RAM_Usage[i]}\n")
        
        self.get_logger().info(f"Data saved to {file_name}")

def main(args=None):
    # Initialize the ROS 2 system
    rclpy.init(args=args)

    # Create an instance of the SaveAirplaneInformation class
    save_performance_metrics = SavePerformanceMetrics()

    try:
        # Spin the node to keep it alive and processing callbacks
        rclpy.spin(save_performance_metrics)
    finally:
        # Save the data before shutting down
        save_performance_metrics.save_performance_data()
        # Destroy the node explicitly
        save_performance_metrics.destroy_node()
        # Shutdown the ROS 2 system
        rclpy.shutdown()

if __name__ == '__main__':
    main()
            

    

    
    