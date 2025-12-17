import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import os
import signal 
import time

class StopSimulation(Node):
    def __init__(self):
        super().__init__("stop_simulation")
        self.subscription = self.create_subscription(
            Bool,
            '/stop_simulation',
            self.stop_simulation_callback,
            10
        )
    
    def stop_simulation_callback(self,msg):
        if msg.data:
            self.get_logger().info("Shutdown signal received! Exiting...")
            os._exit(0)  # Forcefully stop the node

def main(args=None):
    rclpy.init(args=args)
    node = StopSimulation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()