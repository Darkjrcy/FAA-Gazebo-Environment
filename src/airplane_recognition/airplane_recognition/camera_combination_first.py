import rclpy 
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import Trigger
from std_msgs.msg import Float32
import cv2
import numpy as np
# Use custom messages into the file:
from custom_msgs.msg import Yolov11Inference360, InferenceResult360
# Import the YOLO library to load the detection model:
from ultralytics import YOLO
# Initialize the bridge between ROS2 and CV
bridge = CvBridge()

class CameraCombination(Node):
    # Initialize the node of airplane recognition:
    def __init__(self) -> None:
        super().__init__('camera_combination')

        # Load the pre-trained YOLOv8 object detection:
        self.model = YOLO('/home/adcl/AirplanePathFollower/src/airplane_recognition/data/yolo11m.pt')
        self.yolov11_inference = Yolov11Inference360()

        # Use Callback Group to call each subscription to subscribe to the 5 images in parallel:
        self.group1 = MutuallyExclusiveCallbackGroup()
        self.group2 = MutuallyExclusiveCallbackGroup()
        self.group3 = MutuallyExclusiveCallbackGroup()
        self.group4 = MutuallyExclusiveCallbackGroup()
        self.group5 = MutuallyExclusiveCallbackGroup()
        self.group6 = MutuallyExclusiveCallbackGroup()
        
        # Generate the subscription to the cameras in the UAV:
        self.UAV_frontal_cam_img_sub = self.create_subscription(
            Image,
            'frontal_camera/image_raw',
            self.frontal_camera_callback,
            1,
            callback_group=self.group1)
        self.UAV_rigth_frontal_cam_img_sub = self.create_subscription(
            Image,
            'rigth_frontal_camera/image_raw',
            self.rigth_frontal_camera_callback,
            1, 
            callback_group=self.group2)
        self.UAV_rigth_back_cam_img_sub = self.create_subscription(
            Image,
            'rigth_back_camera/image_raw',
            self.rigth_back_camera_callback,
            1, 
            callback_group=self.group3)
        self.UAV_left_back_cam_img_sub = self.create_subscription(
            Image,
            'left_back_camera/image_raw',
            self.left_back_camera_callback,
            1, 
            callback_group=self.group4)
        self.UAV_left_frontal_cam_img_sub = self.create_subscription(
            Image,
            'left_frontal_camera/image_raw',
            self.left_frontal_camera_callback,
            1, 
            callback_group=self.group5)
        
        # Generate the image publisher of the 360 image:
        self.img_pub = self.create_publisher(Image, "/total_image_view", 1)
        # Generate the publisher of the YOLOv11 infernce:
        self.yolov11_pub = self.create_publisher(Yolov11Inference360,"/YOLOv11_inference",10)
        # Generate a publisher of the score values
        self.score_pub = self.create_publisher(Float32,'/airplane_1/score',1)

        # Generate a timer to update the Image Publication and Recognition:
        self.timer = self.create_timer(1/10,self.timer_callback,callback_group=self.group6)

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
    
    # Funtion to be call to change the format of all the images to the cv2 format
    def frontal_camera_callback(self, msg:Image) -> None:
        
        # Change the image from a ROS2 message to a bgr8 format:
        self.frontal_img = bridge.imgmsg_to_cv2(msg,"bgr8")

    # Funtion to be call to change the format of all the images to the cv2 format
    def rigth_frontal_camera_callback(self, msg:Image) -> None:
        
        # Change the image from a ROS2 message to a bgr8 format:
        self.rigth_frontal_img = bridge.imgmsg_to_cv2(msg,"bgr8")
    
    # Funtion to be call to change the format of all the images to the cv2 format
    def rigth_back_camera_callback(self, msg:Image) -> None:
        
        # Change the image from a ROS2 message to a bgr8 format:
        self.rigth_back_img = bridge.imgmsg_to_cv2(msg,"bgr8")

     # Funtion to be call to change the format of all the images to the cv2 format
    def left_frontal_camera_callback(self, msg:Image) -> None:
        
        # Change the image from a ROS2 message to a bgr8 format:
        self.left_frontal_img = bridge.imgmsg_to_cv2(msg,"bgr8")
    
    # Funtion to be call to change the format of all the images to the cv2 format
    def left_back_camera_callback(self, msg:Image) -> None:
        
        # Change the image from a ROS2 message to a bgr8 format:
        self.left_back_img = bridge.imgmsg_to_cv2(msg,"bgr8")
    
    # FUntion to stitch images:
    def stitch_images(self, group_of_images):
        # Get image width and height
        img_height, img_width, _ = group_of_images[0].shape
        # Calculate pixels to crop (20 degrees on each side = 1/4 of image width)
        crop_px = int((5 / 80) * img_width)
        #Crop the images:
        cropped_images=[
            group_of_images[0][:, crop_px:img_width - crop_px],  
            group_of_images[1][:, crop_px:img_width - crop_px],  
            group_of_images[2][:, crop_px:img_width - crop_px],  
            group_of_images[3][:, crop_px:img_width - crop_px],  
            group_of_images[4][:, crop_px:img_width - crop_px]  
        ]
        # Simplifies stiched images:
        try:
            # Horizontally concatenate images
            stitched_image = np.hstack(cropped_images)
            return stitched_image
        except Exception as e:
            return None
    
    def split_stitched_image_for_YOLO(self,image,sliced_width = 2360):
        # Get image width and height
        img_height, img_width, _ = image.shape
        slices = []
        for x in range(0,img_width,sliced_width):
            if x + sliced_width > img_width:
                break
            img_slice = image[:,x:x+sliced_width]
            slices.append(img_slice)
        return slices
    
    def detect_from_slices(self,image):
        slices = self.split_stitched_image_for_YOLO(image)
        # Define the header and the time_stamp of the Yolov11Inference
        self.yolov11_inference.header.frame_id = "camera1_360"
        self.yolov11_inference.header.stamp =  self.get_clock().now().to_msg()
        # Use the model to see through all the images:
        results = self.model(slices,classes=[4])
        # Get the processing times: 
        timing_info = results[0].speed
        self.yolov11_inference.preprocess_time = timing_info['preprocess']  # ms
        self.yolov11_inference.inference_time = timing_info['inference']  # ms
        self.yolov11_inference.postprocess_time = timing_info['postprocess']  # ms
        all_results = []  # To store annotated slices
        Count = 1 # Check the camera where the airplane was found:
        # Forloop between the objects to see whcich are airplanes:
        for r in results:
            boxes = r.boxes
            # Check the results
            for box in boxes:
                self.inf_result = InferenceResult360()
                # get box coordinates in (top, left, bottom, right) format
                b = box.xyxy[0].to('cpu').detach().numpy().copy()  
                c = box.cls
                prob = box.conf.item()
                class_name = self.model.names[int(c)]
                # Tell the camera name:
                if Count == 1:
                    self.inf_result.camera_name = "frontal_camera"
                elif Count == 2 or Count == 3:
                    self.inf_result.camera_name = "rigth_front_camera"
                elif Count == 4:
                    self.inf_result.camera_name = "rigth_back_camera"
                elif Count == 5 or Count == 6:
                    self.inf_result.camera_name = "left_back_camera"
                elif Count == 7:
                    self.inf_result.camera_name = "left_front_camera"
                # Update the other characteristics of the detection:
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
            Count += 1
        # Stitch annotated slices together
        if all_results:
            annotated_frame = np.hstack(all_results)
        else:
            annotated_frame = image  # If no detections, return original image
        # Send the image where the airplane is recognized with a msg
        img_msg = bridge.cv2_to_imgmsg(annotated_frame, encoding="bgr8")
        self.img_pub.publish(img_msg)

        # # Save the image to do new ideas:
        # save_path = "/home/adcl/AirplanePathFollower/DATA/Videos/360Camera/results_yolo11x.jpg"
        # # Save the stitched image
        # cv2.imwrite(save_path, annotated_frame)




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



    # Combine and detect Airplanes form the images using a timer callback:
    def timer_callback(self):
        self.get_logger().info(f"Timer fired. active={self.active}")
        if not hasattr(self, 'frontal_img') or not hasattr(self, 'rigth_frontal_img') or \
        not hasattr(self, 'rigth_back_img') or not hasattr(self, 'left_frontal_img') or \
        not hasattr(self, 'left_back_img'):
            return

        images = [self.frontal_img, self.rigth_frontal_img, self.rigth_back_img,
                self.left_back_img, self.left_frontal_img]

        # Stitch the images
        stitched_image = self.stitch_images(images)

        if not self.active:
            img_msg = bridge.cv2_to_imgmsg(stitched_image, encoding="bgr8")
            self.img_pub.publish(img_msg)
            return
    
        # Use Yolo to detect the airplanes:
        self.detect_from_slices(stitched_image)


        




def main(args=None) -> None:
    rclpy.init(args=args)
    try: 
        camera_combination = CameraCombination()
        executor = MultiThreadedExecutor(num_threads=6)
        executor.add_node(camera_combination)
        try:
            executor.spin()
        finally:
            executor.shutdown()
            camera_combination.destroy_node()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()


    




