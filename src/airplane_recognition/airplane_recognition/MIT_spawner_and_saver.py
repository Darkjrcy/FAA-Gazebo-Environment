# This code is done so the intruder and the ownship moves to the position and orientation of the MIT tracks defined in MIT_filtering
# waiting a small time to detect the system. Then, the information is saved over the MIT /home/adcl/AirplanePathFollower/DATA/MIT_Recognition/Test_1
# fodler.


# Import ROS2 Libraries
from asyncio import FastChildWatcher
import rclpy
from rclpy.node import Node
import threading

# Import ROS2 odometry and standard msg
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, Bool
from gazebo_msgs.srv import SetEntityState
from std_srvs.srv import Trigger
from custom_msgs.msg import Yolov11Inference360

# Import math libraries
import numpy as np
import math

# Import the connection library and json readers
import os
import json
import time

# Libraries to find the yolo models and the MIT filtering points for the detection: 
from ament_index_python.packages import get_package_share_directory

# Library to genrate the required quaternian orientation:
from scipy.spatial.transform import Rotation as R

class SaveAndDetectMITTracks(Node):
    def __init__(self):
        # Start the node:
        super().__init__('save_and_detect_MIT_info')

        # Declare input arguments:
        self.declare_parameter('airplane_name_own', 'airplane_1')
        self.declare_parameter('airplane_name_in', 'airplane_2')
        self.declare_parameter('output_folder', '/home/adcl/AirplanePathFollower/DATA/MIT_Recognition/Test_1')
        self.declare_parameter('fog_type', 'exponential')
        self.declare_parameter('fog_density', 0.01)
        self.declare_parameter('camera_noise_type', 'Gaussian')
        self.declare_parameter('camera_noise_std', 0.01)
        self.declare_parameter('camera_resolution',"3776x2360")
        self.declare_parameter('clutter',"Hanscom_Air_Force_Base_Sourrandings")
        self.declare_parameter('Yolo_model',"Yolo_n")
        # Get the parameters:
        self.own_airplane = self.get_parameter('airplane_name_own').value
        self.in_airplane = self.get_parameter('airplane_name_in').value
        self.output_folder = self.get_parameter('output_folder').value
        self.fog_type = self.get_parameter('fog_type').value
        self.fog_density = self.get_parameter('fog_density').value
        self.camera_noise_type = self.get_parameter('camera_noise_type').value
        self.camera_noise_standard_deviation = self.get_parameter('camera_noise_std').value
        self.camera_resultion = self.get_parameter('camera_resolution').value
        self.clutter = self.get_parameter('clutter').value
        self.YOLO_model = self.get_parameter('Yolo_model').value

        # Define the service that starts the YOLO recognition
        self.start_detection_client = self.create_client(Trigger, f"{self.own_airplane}_start_detection") 
        self.stop_detection_client = self.create_client(Trigger, f"{self.own_airplane}_stop_detection") 

        # Create the client to move the model in the Gazebo world
        self.move_the_airplane_client = self.create_client(SetEntityState, '/gazebo/set_entity_state') # Remember to add the gazebo_ros_state plugin to the world to make it work

        
        # OPen the file containing a dictionary of: "time", "encounter_number","x_own","y_own","z_own","yaw_own","pitch_own","roll_own", 
        # and the same for the intruder "x_in","y_in","z_in","yaw_in","pitch_in","roll_in". 
        package_share_directory = get_package_share_directory('airplane_recognition')
        Waypoints_path = os.path.join(package_share_directory, 'MIT_filtering', 'filtered_points.json')# Important if you want to do your own Waypoints change the direction to yours and follow the smae format
        # Open the file:
        try:
            with open(Waypoints_path, 'r') as file:
                try:
                    Waypoint_data = json.load(file)
                except json.JSONDecodeError:
                    print("Error: File contains invalid JSON")
        except FileNotFoundError:
            print("Error: File not found")
        except Exception as e:
            print(f"An unexpected error occurred: {e}")
        # Define the encounters number:
        self.encounter_number = Waypoint_data["encounter_number"]
        self.time = Waypoint_data["time"]
        # Define the own minimum range position and orientation of the encounters:
        self.x_own = Waypoint_data["x_own"]
        self.y_own = Waypoint_data["y_own"]
        self.z_own = Waypoint_data["z_own"]
        self.roll_own = Waypoint_data["roll_own"]
        self.pitch_own = Waypoint_data["pitch_own"]
        self.yaw_own = Waypoint_data["yaw_own"]
        # The same for the intruder:
        self.x_in = Waypoint_data["x_in"]
        self.y_in = Waypoint_data["y_in"]
        self.z_in = Waypoint_data["z_in"]
        self.roll_in = Waypoint_data["roll_in"]
        self.pitch_in = Waypoint_data["pitch_in"]
        self.yaw_in = Waypoint_data["yaw_in"]

        # Make the variables needed for the detection parametersof  each camera:
        self.front_camera_scores = []
        self.rigth_front_camera_scores = []
        self.rigth_back_camera_scores = []
        self.left_back_camera_scores = []
        self.left_front_camera_scores = []
        self.preprocess_times = []
        self.inference_times = []
        self.postprocess_times = []

        # Lists to save ranges:
        self.ranges = []
        self.time_step = 0
        self.airplane_recog_sub = self.create_subscription(
            Yolov11Inference360,
            f"/{self.own_airplane}/YOLOv11_inference",
            self.score_airplane_callback,
            10
        )
        # Create a publisher to stop the simulation
        self.stop_pub = self.create_publisher(Bool,'/stop_simulation',1)
        # Event to wait for message reception
        self.msg_received_event = threading.Event()


    
    # Save the scores saved depending on the camera that detects it
    def score_airplane_callback(self, msg):
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

    # Funciton to spawn the airplanes in a position and with an orientation:
    def call_service_change_position(self, client, service_name: str, entity_name: str,x: float, y: float, z: float, roll: float, pitch: float, yaw: float) -> bool:
        # Set the model name and the posiiton
        req = SetEntityState.Request()
        req.state.name = entity_name
        req.state.pose.position.x = x
        req.state.pose.position.y = y
        req.state.pose.position.z = z
        # Transform the euler angels into quaternions:
        quat = R.from_euler('zyx',[yaw,-pitch,roll]).as_quat()
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
    
    # Generate a code as the yolo message is not being obtianed
    def wait_for_message(self, timeout=10):
        self.get_logger().info("Waiting for YOLOv11 message...")
        self.msg_received_event.clear()  # Clear the event before waiting
        time.sleep(self.time_step)

        start_time = time.time()
        while not self.msg_received_event.is_set():
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start_time > timeout:
                self.get_logger().warning("Timeout: No YOLOv11 message received")
                break
        self.get_logger().info("YOLOv11 message received or timeout passed")
    
    def move_around_the_airpalnes(self):
        # Do a forloop for all the required trajectories:
        for i in range(len(self.encounter_number)):
            self.get_logger().info(f"Spawner number: {i}, Encounter number: {self.encounter_number[i]}")
            if i == 0:
                self.time_step = 20
            else:
                self.time_step = 10
            
            # Spawn the airplane 1 in position:
            if not self.call_service_change_position(self.move_the_airplane_client, '/gazebo/set_entity_state',self.own_airplane,self.x_own[i]*0.3048,
                                                     self.y_own[i]*0.3048,self.z_own[i]*0.3048,self.roll_own[i],self.pitch_own[i],self.yaw_own[i]):
                        return
            # Spawn the airplane 2 in position:
            if not self.call_service_change_position(self.move_the_airplane_client, '/gazebo/set_entity_state',self.in_airplane,self.x_in[i]*0.3048,
                                                     self.y_in[i]*0.3048,self.z_in[i]*0.3048,self.roll_in[i],self.pitch_in[i],self.yaw_in[i]):
                        return
            # Find the range
            range_ft = np.sqrt((self.y_in[i]-self.y_own[i])**2+(self.x_in[i]-self.x_own[i])**2+(self.z_in[i]-self.z_own[i])**2)
            self.ranges.append(range_ft)
            # Start the detection:
            if not self.call_service_Trigger(self.start_detection_client, f"{self.own_airplane}_start_detection"):
                return
            # Wait for at least one YOLO message
            self.wait_for_message(timeout=10)
            # Save the information in the lists:
            self.preprocess_times.append(self.preprocess_time)
            self.inference_times.append(self.inference_time)
            self.postprocess_times.append(self.postprocess_time)
            self.front_camera_scores.append(self.front_camera_score)
            self.rigth_front_camera_scores.append(self.rigth_front_camera_score)
            self.left_front_camera_scores.append(self.left_front_camera_score)
            self.rigth_back_camera_scores.append(self.rigth_back_camera_score)
            self.left_back_camera_scores.append(self.left_back_camera_score)
            # Stop detection
            if not self.call_service_Trigger(self.stop_detection_client, f"{self.own_airplane}_stop_detection"):
                return
        # Stop teh simulation when the process finish
        msg = Bool()
        msg.data = True
        self.stop_pub.publish(msg)
    
    def save_information(self):
        # Generate the new file folders for the detection data:
        detection_folder_path = os.path.join(self.output_folder, "detection_data")
        os.makedirs(detection_folder_path)

        # Now use it to generate the csv files that we require:
        # The first detection_events that has all the camera specificationcs (noise, resolution, and noise type), the same goes for the fog 
        file_name_detection_events = 'Detection_events.csv'
        file_path_detection_events = os.path.join(detection_folder_path, file_name_detection_events)
        # Save the events file:
        with open(file_path_detection_events,'w') as file:
            file.write(f"Fog_type:,{self.fog_type}\n")
            file.write(f"Fog_density:,{self.fog_density}\n")
            file.write(f"Camera_noise_type:,{self.camera_noise_type}\n")
            file.write(f"Camera_noise_std:,{self.camera_noise_standard_deviation}\n")
            file.write(f"Camera_resolution:,{self.camera_resultion}\n")
            file.write(f"Clutter:,{self.clutter}\n")
            file.write(f"YOLO_model:,{self.YOLO_model}\n")

        # The second one id the detection_process that contains the names of the encounters, range and all the visibility distributions when the airplane change of position.
        file_name_detection_process = 'Detection_process.csv'
        file_path_detection_process = os.path.join(detection_folder_path, file_name_detection_process)
        # Save the detection process file:
        with open(file_path_detection_process, 'w') as file:
            file.write("Encounter_number,Range_ft,Camera1_Confidence_level,Camera2_Confidence_level,Camera3_Confidence_level,Camera4_Confidence_level,Camera5_Confidence_level,Preprocess_time_ms,Inference_time_ms,Postprocess_time_ms\n")
            for i in range(len(self.encounter_number)):
                file.write(f"{self.encounter_number[i]},{self.ranges[i]},{self.front_camera_scores[i]},{self.rigth_front_camera_scores[i]},{self.rigth_back_camera_scores[i]},{self.left_back_camera_scores[i]},{self.left_front_camera_scores[i]},{self.preprocess_times[i]},{self.inference_times[i]},{self.postprocess_times[i]}\n") 

            

            

def main(args=None):
    # Initialize the ROS 2 system
    rclpy.init(args=args)
    # Create an instance of the SaveAirplaneInformation class
    save_and_detect_MIT = SaveAndDetectMITTracks()
    time.sleep(2)
    try:
        save_and_detect_MIT.move_around_the_airpalnes()
        rclpy.spin(save_and_detect_MIT)  # Keep spinning to handle messages
    except Exception as e:
        save_and_detect_MIT.get_logger().error(f'Error during recognition: {e}')
    finally:
        # Save the data before shutting down
        save_and_detect_MIT.save_information()
        # Destroy the node explicitly
        save_and_detect_MIT.destroy_node()
        # Shutdown the ROS 2 system
        rclpy.shutdown()

if __name__ == '__main__':
    main()





        
