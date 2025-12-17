from typing import Dict
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState
from std_srvs.srv import Trigger
from custom_msgs.msg import Yolov11Inference
import numpy as np
import math
import time
from geometry_msgs.msg import Quaternion
import os
import json


class AirplaneRecognitionInFOV(Node):
    def __init__(self) -> None:
        super().__init__('airplane_recognition_in_fov')
        # Make the score of the Recognition:
        self.score = 0.0
        self.detected_objects = ""
        self.x = []
        self.y = []
        self.z = []
        self.scores = []
        # Generate teh subscription to the YLOV11Inference
        self.airplane_recog_sub =self.create_subscription(
            Yolov11Inference,
            '/YOLOv11_inference',
            self.score_airplane_callback,
            1
        )
        # Create the client for the Start detection and Stop detection:
        self.start_detection_client = self.create_client(Trigger, 'start_detection') 
        self.stop_detection_client = self.create_client(Trigger, 'stop_detection') 
        # Create the client to move the model in the Gazebo world
        self.move_the_airplane_client = self.create_client(SetEntityState, '/gazebo/set_entity_state')
    

    

    def score_airplane_callback(self, msg: Yolov11Inference) -> None:
        if not msg.yolov11_inference:
            self.score = 0
        else:
            self.score = msg.yolov11_inference[0].score
    
    def call_service_Trigger(self, client, service_name: str) -> bool:
        req = Trigger.Request()
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None and future.result().success:
            self.get_logger().info(f'{service_name} service call succeeded: {future.result().message}')
            return True
        else:
            self.get_logger().info(f'{service_name} service call failed')
            return False
    
    def euler_to_quaternion(self, roll: float, pitch: float, yaw: float) -> Quaternion:
        """
        Converts Euler angles (in radians) to a Quaternion.
        """
        q = Quaternion()
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy

        return q
    
    def call_service_change_position(self, client, service_name: str, entity_name: str,x: float, y: float, z: float) -> bool:
        req = SetEntityState.Request()
        req.state.name = entity_name
        req.state.pose.position.x = x
        req.state.pose.position.y = y
        req.state.pose.position.z = z
        # Convert -90 degrees to quaternion
        req.state.pose.orientation = self.euler_to_quaternion(0, 0, -math.pi / 2)
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None and future.result().success:
            self.get_logger().info(f'{service_name} service call succeeded')
            return True
        else:
            self.get_logger().info(f'{service_name} service call failed')
            return False

        
    def rotation_matrix(self, axis, theta) -> None:
        """ Returns a 3D rotation matrix for a given axis ('x', 'y', or 'z') and angle theta (in radians). """
        c, s = np.cos(theta), np.sin(theta)
        
        if axis == 'x':
            return np.array([[1, 0, 0],
                            [0, c, -s],
                            [0, s, c]])
        
        elif axis == 'y':
            return np.array([[c, 0, s],
                            [0, 1, 0],
                            [-s, 0, c]])
        
        elif axis == 'z':
            return np.array([[c, -s, 0],
                            [s, c, 0],
                            [0, 0, 1]])
        else:
            raise ValueError("Axis must be 'x', 'y', or 'z'")
    
    def save_results(self, folder_path: str, filename: str) -> None:
        """ Saves x, y, z, and scores lists to a JSON file in a specified folder. """
        
        # Ensure the folder exists
        os.makedirs(folder_path, exist_ok=True)
        
        # Create full file path
        file_path = os.path.join(folder_path, filename)
        
        # Save data as JSON
        data = {
            "x": self.x,
            "y": self.y,
            "z": self.z,
            "scores": self.scores
        }
        
        with open(file_path, "w") as file:
            json.dump(data, file, indent=4)
        
        self.get_logger().info(f"Results saved to {file_path}")

    
    
    def move_around_the_plane(self) -> None:
        # Design the lenghts that you whant the airplane to move
        lengths = [20,40,60,80,100,120,140,160]
        # Define the FOV characteristics:
        heigth = 2160
        width = 4200
        HFOV = 80*np.pi/180
        VFOV = 2*math.atan((heigth/width)*math.tan(HFOV/2))
        # Airplane Size:
        Airplane_width = 6
        Airplane_height = 2
        # Position of the AC 1:
        x0 = np.array([0,0,150])
        # Generate the rotation matrix to go to the upper corner:
        R1 = self.rotation_matrix('x',VFOV/2) @ self.rotation_matrix('z',HFOV/2)
        # Initiallize the forloop to move the airplane around
        for length in lengths:
            # Define the arc sizes in the camera plane:
            hor_arc = HFOV * length
            ver_arc = VFOV *length
            # Generate the initial vector that is going to rotate:
            xH = np.array([0,length,0])
            # Rotate it to the left-upper corner of the FOV
            x1 = R1 @ xH
            # Add the size of the airplane so it enters inside the image
            x1 = x1 + np.array([Airplane_width/2,0,-Airplane_height/2])

            # After the first change of the position usign set_entity_state of GAZEBO
            if not self.call_service_change_position(self.move_the_airplane_client, '/gazebo/set_entity_state',"airplane_2",x1[0]+x0[0],x1[1]+x0[1],x1[2]+x0[2]):
                return
            # Get the first position:
            self.x.append(x1[0]+x0[0])
            self.y.append(x1[1]+x0[1])
            self.z.append(x1[2]+x0[2])
            # Start the detection:
            if not self.call_service_Trigger(self.start_detection_client, 'start_detection'):
                return
            # Wait time so the system is stable:
            time.sleep(1)
            # Get the score
            self.scores.append(self.score)
            # Stop detection
            if not self.call_service_Trigger(self.stop_detection_client, 'stop_detection'):
                return
            
            # NUmber of sub division:
            n = 5
            # Divide the segments that the airplane is going to move:
            hor_div = int(((hor_arc/Airplane_width)//n)+1)
            ver_div = int(((ver_arc/Airplane_height)//n)+1)
            # Generate the step of the angles to rotate the position vector.
            d_HFOV = HFOV/(hor_div*n)
            d_VFOV = VFOV/(ver_div*n)
            # Check the scores value:
            idx_cap = len(self.scores)
            # Initiallize the forloop to rotate around the z and x axis to change the position
            for x_rot in range(n*ver_div-1):
                x2 = x1
                for y_rot in range(n*hor_div-1):
                    # Rotate respect to the z axis:
                    x2 = self.rotation_matrix('z',-d_HFOV) @ x2
                    # Update the values of the lsit in the x,y,z position:
                    self.x.append(x2[0]+x0[0])
                    self.y.append(x2[1]+x0[1])
                    self.z.append(x2[2]+x0[2])

                    # After the rotation in the HFOV change the position usign set_entity_state of GAZEBO
                    if not self.call_service_change_position(self.move_the_airplane_client, '/gazebo/set_entity_state',"airplane_2",x2[0]+x0[0],x2[1]+x0[1],x2[2]+x0[2]):
                        return
                    # Start the detection:
                    if not self.call_service_Trigger(self.start_detection_client, 'start_detection'):
                        return
                    # See the length
                    print(f"The length done is {length}")   
                    # Wait time so the system is stable:
                    time.sleep(1)
                    # Get the score
                    self.scores.append(self.score)
                    # Stop detection
                    if not self.call_service_Trigger(self.stop_detection_client, 'stop_detection'):
                        return
                
                # Rotate vertically respecto the x axis 
                x1 = self.rotation_matrix('x',-d_VFOV) @ x1
                # Update the values of the lsit in the x,y,z position:
                self.x.append(x1[0]+x0[0])
                self.y.append(x1[1]+x0[1])
                self.z.append(x1[2]+x0[2])
                

                # After the rotation in the VFOV change the position usign set_entity_state of GAZEBO
                if not self.call_service_change_position(self.move_the_airplane_client, '/gazebo/set_entity_state',"airplane_2",x1[0]+x0[0],x1[1]+x0[1],x1[2]+x0[2]):
                    return
                # Start the detection:
                if not self.call_service_Trigger(self.start_detection_client, 'start_detection'):
                    return
                # Wait time so the system is stable:
                time.sleep(1)
                # Get the score
                self.scores.append(self.score)
                # Stop detection
                if not self.call_service_Trigger(self.stop_detection_client, 'stop_detection'):
                    return
            # Break the forloop in case the values obtained in the score for the UAV are 0 or the coed is not the detecting the Airplane at least 40 times
            if all(value <= 0.32 for value in self.scores[idx_cap:]):
                break
        


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AirplaneRecognitionInFOV()
    try:
        node.move_around_the_plane()
    except Exception as e:
        node.get_logger().error(f'Error during recognition: {e}')
    finally:
        # Save the information into a json file:
        node.save_results("/home/adcl/AirplanePathFollower/src/airplane_recognition/Recognition_results/Noisy_data/", "airplane_detection_noise_new.json")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
                    





