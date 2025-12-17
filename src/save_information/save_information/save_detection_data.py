# Import ROS2 Libraries
import rclpy
from rclpy.node import Node

# Import ROS2 odometry msg
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from custom_msgs.msg import Yolov11Inference

# Import math libraries
import numpy as np
import math
import threading

# Import the connection library
import os


class SaveDetectionInformation(Node):
    def __init__(self):
        # Start the node:
        super().__init__('save_detection_information')

        # Declare parameters
        self.declare_parameter('airplane_name_avoider', 'airplane_1')
        self.declare_parameter('airplane_name_obstacle', 'airplane_2')
        self.declare_parameter('output_folder', '/home/adcl/AirplanePathFollower/DATA/Fligths/FLight1/detection_data')
        self.declare_parameter('frecuency', 10)

        # Get the parameters:
        self.airplane_avoider = self.get_parameter('airplane_name_avoider').value
        self.airplane_obstacle = self.get_parameter('airplane_name_obstacle').value
        self.output_folder = self.get_parameter('output_folder').value
        self.frequency = self.get_parameter('frecuency').value

        # Define a variable that checks if the simulation started:
        self.simulation_started = False
        self.detection_started = False

        # Add a counter to start when the simulation started and take out the delay time values:
        self.initial_real_time = 0
        self.count_real = 0
        self.initial_sim_time = 0
        self.count_sim = 0
        self.count = 0

        # Nan in case there isn't an avoidance during the fligth
        self.First_detection_time = 'Nan'
        
        # Generate the lists to add the information needed:
        self.REAL_TIME_s = []
        self.SIM_TIME_s = []
        self.Range_ft = []
        self.Bearing_UAV_deg = []
        self.Bearing_Target_deg = []
        
        # Make the variables needed for the detection parametersof  each camera:
        self.front_camera_scores = []
        self.rigth_front_camera_scores = []
        self.rigth_back_camera_scores = []
        self.left_back_camera_scores = []
        self.left_front_camera_scores = []
        self.preprocess_times = []
        self.inference_times = []
        self.postprocess_times = []

        # Generate the subscriptions:
        self.odom_avoider_sub = self.create_subscription(Odometry, f'/{self.airplane_avoider}/odom', self.odom_avoider_callback, 10)
        self.odom_obstacle_sub = self.create_subscription(Odometry, f'/{self.airplane_obstacle}/odom', self.odom_obstacle_callback, 10)
        self.odom_detection_sub = self.create_subscription(Yolov11Inference, '/YOLOv11_inference', self.detection_callback, 10)
        self.real_time_sub = self.create_subscription(Float32, '/real_time', self.real_time_callback, 10)
        self.sim_time_sub = self.create_subscription(Float32, '/sim_time', self.sim_time_callback, 10)
        # Get the time at which the avoidance starts:
        self.avoidance_sub = self.create_subscription(Float32, '/Avoidance', self.avoidance_callback, 10)

        # Generate a timer to save the information:
        self.timer = self.create_timer(1/self.frequency,self.timer_callback)

        # Event to wait for message reception
        self.msg_received_event = threading.Event()


    
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
    
    def odom_avoider_callback(self,msg):
        vel_x = msg.twist.twist.linear.y
        vel_y = msg.twist.twist.linear.x
        vel_z = -msg.twist.twist.linear.z
        vel = math.sqrt(vel_x**2 + vel_y**2 + vel_z**2)
        # Find if the simulation started
        if vel > 0.5:
            self.simulation_started = True
        
        # Save the needed data:
        if self.simulation_started:
            self.x_avoider = msg.pose.pose.position.y / 0.3048
            self.y_avoider=msg.pose.pose.position.x / 0.3048
            self.z_avoider=-msg.pose.pose.position.z / 0.3048
            # Get the bearing angle:
            quaternion = (
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w,
            )
            roll, pitch, yaw = self.quaternion_to_euler(quaternion)
            self.avoider_bearig_deg=-math.degrees(yaw) + 90
    
    def odom_obstacle_callback(self,msg):
        
        # Save the needed data:
        if self.simulation_started:
            self.x_obstacle = msg.pose.pose.position.y / 0.3048
            self.y_obstacle=msg.pose.pose.position.x / 0.3048
            self.z_obstacle=-msg.pose.pose.position.z / 0.3048
            # Get the bearing angle:
            quaternion = (
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w,
            )
            roll, pitch, yaw = self.quaternion_to_euler(quaternion)
            self.obstacle_bearig_deg=-math.degrees(yaw) + 90
    
    def detection_callback(self,msg):
        self.front_camera_score = 0
        self.rigth_front_camera_score = 0
        self.rigth_back_camera_score = 0
        self.left_back_camera_score = 0
        self.left_front_camera_score = 0
        if not msg.yolov11_inference:
            self.preprocess_time = 0
            self.inference_time = 0
            self.postprocess_time = 0
        else:
            self.preprocess_time = msg.preprocess_time
            self.inference_time = msg.inference_time
            self.postprocess_time = msg.postprocess_time
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
        # Signal that a message was received
        self.msg_received_event.set()


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
    
    def avoidance_callback(self,msg):
        if msg.data == 1.0 and self.count == 0:
            self.First_detection_time = self.time_s
            self.count = 1

    
    def timer_callback(self):
        if self.simulation_started:
            self.REAL_TIME_s.append(self.time_s)
            self.SIM_TIME_s.append(self.time_sim_s)
            self.Range_ft.append(math.sqrt((self.x_avoider-self.x_obstacle)**2+(self.y_avoider-self.y_obstacle)**2+(self.z_avoider-self.z_obstacle)**2))
            self.Bearing_UAV_deg.append(self.avoider_bearig_deg)
            self.Bearing_Target_deg.append(self.obstacle_bearig_deg)
            self.preprocess_times.append(self.preprocess_time)
            self.inference_times.append(self.inference_time)
            self.postprocess_times.append(self.postprocess_time)
            self.front_camera_scores.append(self.front_camera_score)
            self.rigth_front_camera_scores.append(self.rigth_front_camera_score)
            self.left_front_camera_scores.append(self.left_front_camera_score)
            self.rigth_back_camera_scores.append(self.rigth_back_camera_score)
            self.left_back_camera_scores.append(self.left_back_camera_score)

    
    def save_detection_data(self):
        # Save the detection information:
        file_name_detection = 'Detection_process.csv'
        file_path_detection = os.path.join(self.output_folder, file_name_detection)

        # Save the information:
        with open(file_path_detection, 'w') as file:
            file.write("Real_time_s,Simulation_time_s,Range_ft,Bearing_Avoider_deg,Bearing_obstacle_deg,Camera1_Confidence_level,Camera2_Confidence_level,Camera3_Confidence_level,Camera4_Confidence_level,Camera5_Confidence_level,Preprocess_time_ms,Inference_time_ms,Postprocess_time_ms\n")
            for i in range(len(self.REAL_TIME_s)):
                file.write(f"{self.REAL_TIME_s[i]},{self.SIM_TIME_s[i]},{self.Range_ft[i]},{self.Bearing_UAV_deg[i]},{self.Bearing_Target_deg[i]},{self.front_camera_scores[i]},{self.rigth_front_camera_scores[i]},{self.rigth_back_camera_scores[i]},{self.left_back_camera_scores[i]},{self.left_front_camera_scores[i]},{self.preprocess_times[i]},{self.inference_times[i]},{self.postprocess_times[i]}\n") 

        
        # Save the Detection events:
        file_name_events = 'Detection_events.csv'
        file_path_events = os.path.join(self.output_folder, file_name_events)

        with open(file_path_events, 'w') as file:
            file.write(f"Starting_avoidance_time(s):, {self.First_detection_time}\n")
            min_range = min(self.Range_ft)
            file.write(f"Minimum_range(ft):,{min_range}\n")
            min_time_idx=self.Range_ft.index(min_range)
            file.write(f"Time_to_closest_approach(s):,{self.REAL_TIME_s[min_time_idx]}\n")
        

        self.get_logger().info(f"Data saved to {self.output_folder}")



def main(args=None):
    # Initialize the ROS 2 system
    rclpy.init(args=args)

    # Create an instance of the SaveAirplaneInformation class
    save_detection_info = SaveDetectionInformation()

    try:
        # Spin the node to keep it alive and processing callbacks
        rclpy.spin(save_detection_info)
    finally:
        # Save the data before shutting down
        save_detection_info.save_detection_data()
        # Destroy the node explicitly
        save_detection_info.destroy_node()
        # Shutdown the ROS 2 system
        rclpy.shutdown()

if __name__ == '__main__':
    main()


