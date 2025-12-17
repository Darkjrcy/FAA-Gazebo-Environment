from ast import arg
from ultralytics import YOLO
import rclpy 
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_srvs.srv import Trigger
from std_msgs.msg import Float32
# Use custom messages into the file:
from custom_msgs.msg import Yolov11Inference, InferenceResult
# Initialize the bridge between ROS2 and CV
bridge = CvBridge()

# Libraries to find the yolo models and the MIT filtering points for the detection: 
from ament_index_python.packages import get_package_share_directory
import os 

class AirplaneDetection(Node):
    # Initialize the node of airplane recognition:
    def __init__(self) -> None:
        super().__init__('airplane_detection')
    
        # Load the pre-trained YOLOv8 object detection:
        package_share_directory = get_package_share_directory('airplane_recognition')
        model_path = os.path.join(package_share_directory, 'data', 'yolo11m.pt')
        self.model = YOLO(model_path)
        self.yolov11_inference = Yolov11Inference()
        
        # Generate the subscription to the camera in the UAV:
        self.UAV_cam_img_sub = self.create_subscription(
            Image,
            'camera1/image_raw',
            self.camera_callback,
            10)
        
        # Generate the publisher for the Object detection:
        self.img_pub = self.create_publisher(Image, "/Camera_detection", 1)

        # Generate the publisher of the YOLOv11 infernce:
        self.yolov11_pub = self.create_publisher(Yolov11Inference,"/YOLOv11_inference",1)
        # Generate a publisher of the score values
        self.score_pub = self.create_publisher(Float32,'/airplane_1/score',1)


        # Generate the service to start detecting the airplanes:
        self.start_service = self.create_service(Trigger, 'start_detection', self.start_detection)
        self.stop_service = self.create_service(Trigger, 'stop_detection', self.stop_detection)

        # Initialize the active varible to start the detection
        self.active = False
    
    # Function to start the service and start the detection
    def start_detection(self, request, response):
        self.active = True
        response.success = True
        response.message = "Airplane detection started"
        return response
    
    # Function to stop the detection using hte service
    def stop_detection(self,request,response):
        self.active = False
        response.success = True
        response.message = "Airplane detection stopped"
        return response
    
    def split_stitched_image_for_YOLO(self,image):
        # Get image width and height
        img_height, img_width, _ = image.shape
        sliced_width = img_width//2
        slices = []
        for x in range(0,img_width,sliced_width):
            if x + sliced_width > img_width:
                break
            img_slice = image[:,x:x+sliced_width]
            slices.append(img_slice)
        return slices
    
    
    
    # FUnction that is going to be call each time the the node receibes information:
    def camera_callback(self, msg:Image) -> None:
        
        if not self.active:
            return
        
        # Change the image from a ROS2 message to a bgr8 format:
        img = bridge.imgmsg_to_cv2(msg,"bgr8")
        slices = self.split_stitched_image_for_YOLO(img)
        
        # Use the YOLO model to reconize the objects in the picture: 
        results = self.model(slices,classes=[4])
        all_results = []

        # Get the processing times: 
        timing_info = results[0].speed
        self.yolov11_inference.preprocess_time = timing_info['preprocess']  # ms
        self.yolov11_inference.inference_time = timing_info['inference']  # ms
        self.yolov11_inference.postprocess_time = timing_info['postprocess']  # ms


        # Put the header for the interface
        self.yolov11_inference.header.frame_id = "camera_1"
        self.yolov11_inference.header.stamp =  self.get_clock().now().to_msg()

        # Forloop between the objects to see whcich are airplanes:
        for r in results:
            boxes = r.boxes
            for box in boxes:
                self.inf_result = InferenceResult()
                # get box coordinates in (top, left, bottom, right) format
                b = box.xyxy[0].to('cpu').detach().numpy().copy()  
                c = box.cls
                prob = box.conf.item()
                class_name = self.model.names[int(c)]
                self.inf_result.class_name = class_name
                self.inf_result.left = int(b[0])
                self.inf_result.top = int(b[1])
                self.inf_result.right = int(b[2])
                self.inf_result.bottom = int(b[3])
                self.inf_result.box_width = (self.inf_result.right - self.inf_result.left) 
                self.inf_result.box_height = (self.inf_result.bottom - self.inf_result.top)
                self.inf_result.x = self.inf_result.left + (self.inf_result.box_width/2.0)
                self.inf_result.y = self.inf_result.top + (self.inf_result.box_height/2.0)
                self.inf_result.score = prob
                self.yolov11_inference.yolov11_inference.append(self.inf_result)
            # Annotated slice:
            annotated_slice = r.plot()
            all_results.append(annotated_slice)
        
        # Stitch annotated slices together
        if all_results:
            annotated_frame = np.hstack(all_results)
        else:
            annotated_frame = img  # If no detections, return original image
        # Send the image where the airplane is recognized with a msg
        img_msg = bridge.cv2_to_imgmsg(annotated_frame, encoding="bgr8")
        self.img_pub.publish(img_msg)

        # Publish the Yolo messgae:
        self.yolov11_pub.publish(self.yolov11_inference)

        # Publish the YOLOv11 Inference information to a ROS2 node in case there are airplanes recognized:
        if self.yolov11_inference.yolov11_inference:
            # Create a Float32 message for the score
            score_msg = Float32()
            score_msg.data = float(self.yolov11_inference.yolov11_inference[0].score)

            # Publish the score message
            self.score_pub.publish(score_msg)
        else:
            score_msg = Float32()
            score_msg.data = float(0.0)
            self.score_pub.publish(score_msg)
        self.yolov11_inference.yolov11_inference.clear()



def main(args=None) -> None:
    rclpy.init(args=args)
    airplane_detection = AirplaneDetection()
    rclpy.spin(airplane_detection)
    airplane_detection.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    





        



