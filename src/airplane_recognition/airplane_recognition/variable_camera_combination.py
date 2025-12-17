from encodings.punycode import T
from tabnanny import verbose
from xmlrpc.client import TRANSPORT_ERROR
import rclpy 
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import Trigger
from std_msgs.msg import Float32
from threading import Lock
import cv2
import numpy as np
# Use custom messages into the file:
from custom_msgs.msg import Yolov11Inference360, InferenceResult360
# Import the YOLO library to load the detection model:
import logging
import torch
from ultralytics import YOLO
from ultralytics.utils import LOGGER
LOGGER.setLevel(logging.ERROR)
# Initialize the bridge between ROS2 and CV
bridge = CvBridge()
# Libraries to find the yolo models and the MIT filtering points for the detection: 
from ament_index_python.packages import get_package_share_directory
import os 

# IDX definition for hte different cameras:
IDX = {
    'front': 0,
    'right_front': 1,
    'right_back': 2,
    'left_back': 3,
    'left_front': 4,
}

CAMERA_NAME_FOR_MSG = {
    'front':        'frontal_camera',
    'right_front':  'rigth_front_camera',
    'right_back':   'rigth_back_camera',
    'left_back':    'left_back_camera',
    'left_front':   'left_frontal_camera',
}

CAMERA_KEYS = ['front', 'right_front', 'right_back', 'left_back', 'left_front']


class VariableCameraCombination(Node):
    # Initialize the node of variable airplane recognition with the different number of cameras:
    def __init__(self) -> None:
        super().__init__('variable_camera_combination')

        # Model name where the cameras are connected:
        self.declare_parameter('model_name', 'airplane_1')
        self.declare_parameter('use_gpu', True)
        self.model_name = self.get_parameter('model_name').value
        self.use_gpu = bool(self.get_parameter('use_gpu').value)

        # Change YOLO to work from the cpu to the gpu:
        has_cuda = torch.cuda.is_available()
        self.device = 'cuda:0' if (has_cuda and self.use_gpu) else 'cpu'
        # Load the pre-trained YOLOv8 object detection:
        package_share_directory = get_package_share_directory('airplane_recognition')
        model_path = os.path.join(package_share_directory, 'data', 'yolo11m.pt')
        self.model = YOLO(model_path)
        self.model.to(self.device)
        # Speeds tweaks for constant size image:
        if self.use_gpu:
            if self.device.startswith('cuda'):
                torch.backends.cudnn.benchmark = True
                try:
                    self.model.overrides['half'] = True
                except Exception:
                    pass
        # Log device:
        dev_name = (torch.cuda.get_device_name(0) if self.device.startswith('cuda') else 'CPU')
        self.get_logger().info(f"YOLO device: {self.device} ({dev_name})")
        # Published message:
        self.yolov11_inference = Yolov11Inference360()

        # Use Callback Group to call each subscription to subscribe to the 5 images in parallel:
        self.sub_group = ReentrantCallbackGroup()
        self.timer_group = MutuallyExclusiveCallbackGroup()

        # List to save the messages:
        self.img_lock = Lock()
        self.images = [None] * 5

        # Gnerate subscribers fo the possible cameras in the UAV:
        self.cam_1_sub = self.create_subscription(
            Image,
            f"{self.model_name}_camera1/image_raw",
            lambda msg: self.set_image(IDX['front'], msg),
            1,
            callback_group=self.sub_group)
        self.cam_2_sub = self.create_subscription(
            Image,
            f"{self.model_name}_camera2/image_raw",
            lambda msg: self.set_image(IDX['right_front'], msg),
            1, 
            callback_group=self.sub_group)
        self.cam_3_sub = self.create_subscription(
            Image,
            f"{self.model_name}_camera3/image_raw",
            lambda msg: self.set_image(IDX['right_back'], msg),
            1, 
            callback_group=self.sub_group)
        self.cam_4_sub = self.create_subscription(
            Image,
            f"{self.model_name}_camera4/image_raw",
            lambda msg: self.set_image(IDX['left_back'], msg),
            1, 
            callback_group=self.sub_group)
        self.cam_5_sub = self.create_subscription(
            Image,
            f"{self.model_name}_camera5/image_raw",
            lambda msg: self.set_image(IDX['left_front'], msg),
            1, 
            callback_group=self.sub_group)
        

        # Generate the publisher of the YOLOv11 infernce:
        self.yolov11_pub = self.create_publisher(Yolov11Inference360,f"/{self.model_name}/YOLOv11_inference",10)

        # Generate a timer to update the Image Publication and Recognition:
        self.timer = self.create_timer(1/10,self.timer_callback,callback_group=self.timer_group)

        # Generate the service to start detecting the airplanes:
        self.start_service = self.create_service(Trigger, f"{self.model_name}_start_detection", self.start_detection)
        self.stop_service = self.create_service(Trigger, f"{self.model_name}_stop_detection", self.stop_detection)
        
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
    
    # Callback used in every image:
    def set_image(self, idx: int, msg: Image):
        img = bridge.imgmsg_to_cv2(msg,"bgr8")
        with self.img_lock:
            self.images[idx] = img

    # Timer callback to analyze each image:
    def timer_callback(self):
        # Snapshot of avalibale frmaes:
        with self.img_lock:
            batch = [(i, im) for i,im in enumerate(self.images) if im is not None]
        
        # Check if the detection was activated:
        if not batch or not self.active:
            return
        
        # Build the batch and detected in YOLO:
        idxs, imgs = zip(*batch)
        if self.use_gpu:
            use_half = self.device.startswith('cuda')
            results = self.model.predict(
                list(imgs),
                classes=[4],
                verbose=False,
                device=self.device,
                half=use_half,  # fp16 on GPU, fp32 on CPU
            )
        else:
            results = self.model(list(imgs), classes=[4], verbose=False)
        
        # Generate the message that is going to be send from YOLO detection
        # Get the processing times: 
        timing_info = results[0].speed
        self.yolov11_inference.preprocess_time = timing_info['preprocess']  # ms
        self.yolov11_inference.inference_time = timing_info['inference']  # ms
        self.yolov11_inference.postprocess_time = timing_info['postprocess']  # ms
        # Define the header and the time_stamp of the Yolov11Inference
        self.yolov11_inference.header.frame_id = "camera1_360"
        self.yolov11_inference.header.stamp =  self.get_clock().now().to_msg()
        # NUmber of boxes: 
        total_boxes = 0
        # Foorloop around the objects and the idx to detect the airplanes and the camera that detct it:
        for r, cam_idx in zip(results, idxs):
            boxes = r.boxes
            cam_key = CAMERA_KEYS[cam_idx]
            cam_name = CAMERA_NAME_FOR_MSG[cam_key]
            for box in boxes:
                total_boxes +=1
                self.inf_result = InferenceResult360()
                # get box coordinates in (top, left, bottom, right) format
                b = box.xyxy[0].to('cpu').detach().numpy().copy()  
                c = box.cls
                prob = box.conf.item()
                class_name = self.model.names[int(c)]

                # Add the imaformation fo detection to the message:
                self.inf_result.camera_name = cam_name
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
        
         # Print only whn it finds something:
        if total_boxes > 0:
            timing = getattr(results[0], "speed", {})
            self.get_logger().info(
                f"Speed: {timing.get('preprocess', 0):.1f}ms preprocess, "
                f"{timing.get('inference', 0):.1f}ms inference, "
                f"{timing.get('postprocess', 0):.1f}ms postprocess; "
                f"Detections: {total_boxes}"
            )
        
        # Publish the message:
        self.yolov11_pub.publish(self.yolov11_inference)

        # Clear the message;
        self.yolov11_inference.yolov11_inference.clear()
    

def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try: 
        node = VariableCameraCombination()
        executor = MultiThreadedExecutor(num_threads=6)
        executor.add_node(node)
        try:
            executor.spin()
        finally:
            executor.shutdown()
            node.destroy_node()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()


        

    
