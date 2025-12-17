# Import ROS2 Libraries
import rclpy
from rclpy.node import Node

# Import ROS2 odometry msg
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32

# Import math libraries
import numpy as np
import math

# Import the connection library
import os

class SaveAirplaneInformation(Node):
    def __init__(self):
        # Start the node:
        super().__init__('save_airplane_information')

        # Declare parameters
        self.declare_parameter('airplane_name', 'default_airplane_name')
        self.declare_parameter('output_folder', 'default_output_folder')
        self.declare_parameter('frecuency', 10)

        # Get parameters
        self.airplane_ID = self.get_parameter('airplane_name').value
        self.output_folder = self.get_parameter('output_folder').value
        self.frequency = self.get_parameter('frecuency').value

        # Log the parameters to verify they are being set correctly
        self.get_logger().info(f"Airplane Name: {self.airplane_ID}")
        self.get_logger().info(f"Output Folder: {self.output_folder}")
        self.get_logger().info(f"Output Folder: {self.frequency}")

        # Define a variable that checks if the simulation started:
        self.simulation_started = False

        # Add a counter to start when the simulation started and take out the delay time values:
        self.initial_real_time = 0
        self.count_real = 0
        self.initial_sim_time = 0
        self.count_sim = 0

        # Generate lsits to add the information for each airplane:
        self.REAL_TIME_s = []
        self.SIM_TIME_s = []
        self.RTF = []
        self.X_ft = []
        self.Y_ft = []
        self.Z_ft = []
        self.VX_ftps = []
        self.VY_ftps = []
        self.VZ_ftps = []
        self.ROLL_deg = []
        self.PITCH_deg = []
        self.YAW_deg = []

        # Define the subscribers to the odometry and the time variables
        self.odom_sub = self.create_subscription(Odometry, f'/{self.airplane_ID}/odom', self.odom_callback, 10)
        self.real_time_sub = self.create_subscription(Float32, '/real_time', self.real_time_callback, 10)
        self.sim_time_sub = self.create_subscription(Float32, '/sim_time', self.sim_time_callback, 10)


        # Generate a timer to save the information:
        self.timer = self.create_timer(1/self.frequency,self.timer_callback)


    def quaternion_to_euler(self, quaternion):
        # Convert quaternion to Euler angles (roll, pitch, yaw)
        x, y, z, w = quaternion
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)

        return roll, pitch, yaw

    def odom_callback(self, msg):
        vel_x = msg.twist.twist.linear.y
        vel_y = msg.twist.twist.linear.x
        vel_z = -msg.twist.twist.linear.z
        vel = math.sqrt(vel_x**2 + vel_y**2 + vel_z**2)
        # Find if the simulation started
        if vel > 0.5:
            self.simulation_started = True

        # Save the values in the vectors:
        if self.simulation_started:
            # Save the position and the velocity:
            self.x_ft=msg.pose.pose.position.y / 0.3048
            self.y_ft=msg.pose.pose.position.x / 0.3048
            self.z_ft=-msg.pose.pose.position.z / 0.3048
            self.vx_ftps=msg.twist.twist.linear.y / 0.3048
            self.vy_ftps=msg.twist.twist.linear.x / 0.3048
            self.vz_ftps=-msg.twist.twist.linear.z / 0.3048

            # Save the angles:
            quaternion = (
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w,
            )
            roll, pitch, yaw = self.quaternion_to_euler(quaternion)
            self.roll_deg=math.degrees(roll)
            self.pitch_deg=-math.degrees(pitch)
            self.yaw_deg=-math.degrees(yaw) + 90

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
            self.X_ft.append(self.x_ft)
            self.Y_ft.append(self.y_ft)
            self.Z_ft.append(self.z_ft)
            self.VX_ftps.append(self.vx_ftps)
            self.VY_ftps.append(self.vy_ftps)
            self.VZ_ftps.append(self.vz_ftps)
            self.ROLL_deg.append(self.roll_deg)
            self.PITCH_deg.append(self.pitch_deg)
            self.YAW_deg.append(self.yaw_deg)



    def save_airplane_data(self):
        file_name = f'{self.airplane_ID}_states.csv'
        file_path = os.path.join(self.output_folder, file_name)

        # Save the information:
        with open(file_path, 'w') as file:
            file.write("Real_time_s,Simulation_time_s,Real_time_factor,x_ft,y_ft,z_ft,vx_ftps,vy_ftps,vz_ftps,roll_deg,pitch_deg,yaw_deg\n")
            for i in range(len(self.X_ft)):
                file.write(f"{self.REAL_TIME_s[i]},{self.SIM_TIME_s[i]},{self.RTF[i]},{self.X_ft[i]},{self.Y_ft[i]},{self.Z_ft[i]},{self.VX_ftps[i]},{self.VY_ftps[i]},{self.VZ_ftps[i]},{self.ROLL_deg[i]},{self.PITCH_deg[i]},{self.YAW_deg[i]}\n")

        
        self.get_logger().info(f"Data saved to {file_name}")

def main(args=None):
    # Initialize the ROS 2 system
    rclpy.init(args=args)

    # Create an instance of the SaveAirplaneInformation class
    save_airplane_info = SaveAirplaneInformation()

    try:
        # Spin the node to keep it alive and processing callbacks
        rclpy.spin(save_airplane_info)
    finally:
        # Save the data before shutting down
        save_airplane_info.save_airplane_data()
        # Destroy the node explicitly
        save_airplane_info.destroy_node()
        # Shutdown the ROS 2 system
        rclpy.shutdown()

if __name__ == '__main__':
    main()