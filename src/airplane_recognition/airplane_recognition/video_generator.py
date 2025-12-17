#!/usr/bin/env python
import re
import os
import rclpy 
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory

# Initialize CV Bridge
bridge = CvBridge()

class VideoGenerator(Node):
    def __init__(self, output_video_path, fps, frame_size):
        # Initialize ROS node
        super().__init__('video_generator_node')

        # Initialize VideoWriter
        self.video_writer = cv2.VideoWriter(output_video_path, cv2.VideoWriter_fourcc(*'MPEG'), fps, frame_size)

        # Check if VideoWriter is opened properly
        if not self.video_writer.isOpened():
            self.get_logger().error("Failed to open video file for writing!")
            raise RuntimeError("Could not open the video file for writing!")

        # Subscribe to the /Camera_detection topic
        self.image_sub = self.create_subscription(Image, '/total_image_view', self.image_callback, 10)

    def image_callback(self, msg):
        try:
            # Convert the ROS2 image message to OpenCV format
            cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")

            # Define the expected frame size explicitly (instead of querying VideoWriter)
            expected_width, expected_height = 16380, 2275

            # Check if the image size matches the expected frame size
            if (cv_image.shape[1], cv_image.shape[0]) != (expected_width, expected_height):
                self.get_logger().warn(f"Resizing frame from {cv_image.shape[1]}x{cv_image.shape[0]} to {expected_width}x{expected_height}")
                cv_image = cv2.resize(cv_image, (expected_width, expected_height))

            # Write the frame to the video file
            self.video_writer.write(cv_image)

        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")


    def video_saver(self):
        if self.video_writer.isOpened():
            self.video_writer.release()
            print("Video saved and resources released.")


def main(args=None):
    # Initialize the ROS 2 system
    rclpy.init(args=args)

    # Output video file path
    # Go up to workspace root (depending where your launch file is!)
    this_file_path = os.path.realpath(__file__)
    this_dir = os.path.dirname(this_file_path)
    # Example: if your launch file is in src/plane_recognition_package/launch/, go up 3 levels:
    workspace_root = os.path.abspath(os.path.join(this_dir, '../../../../..'))

    # Now point to DATA/MIT_Recognition
    output_folder_path = os.path.join(workspace_root, 'DATA', 'Videos','Recognition')

    # Videos inside the folder:
    videos = [f for f in os.listdir(output_folder_path) if os.path.isdir(os.path.join(output_folder_path,f))]

    # Extract the number of videos:
    video_numbers = []
    for video in videos:
        match = re.match(r'Output_video(\d+).mp4',video)
        if match:
            video_numbers.append(int(match.group(1)))
    
    # video_numbers the next fligth folder number
    if video_numbers:
        next_video_number = max(video_numbers) + 1
    else:
        next_video_number = 1
    
    # Generate the video path:
    new_video_name = f'Output_video{next_video_number}'
    output_video_path = os.path.join(output_folder_path,new_video_name+'.mp4')

    # Create an instance of VideoGenerator
    video_gen = VideoGenerator(output_video_path, fps=30, frame_size=(16380, 2275))

    try:
        # Spin the node to keep it alive and processing callbacks
        rclpy.spin(video_gen)
    except KeyboardInterrupt:
        print("\nShutting down gracefully...")
    finally:
        # Release video resources and shutdown node
        video_gen.video_saver()
        video_gen.destroy_node()

        # Ensure ROS is shut down properly
        if rclpy.ok():
            rclpy.shutdown()
        print("ROS2 shutdown complete.")

if __name__ == '__main__':
    main()
