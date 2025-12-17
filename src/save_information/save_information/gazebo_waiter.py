import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from rclpy.qos import qos_profile_system_default
from rclpy.executors import SingleThreadedExecutor

class GazeboWaiter(Node):
    def __init__(self):
        super().__init__('gazebo_waiter')
        self.time_subs = self.create_subscription(
            Float32,
            '/real_time',
            self.callback,
            qos_profile_system_default
        )
        self.ready = False
        self.get_logger().info("Waiting for real_time in Gazebo to be at least 30 (s)")
    
    def callback(self,msg):
        if msg.data > 30:
            self.get_logger().info("Gazebo is ready! Exiting node...")
            self.ready = True
            rclpy.shutdown()
    
def main(args=None):
    rclpy.init(args=args)
    node = GazeboWaiter()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        while rclpy.ok() and not node.ready:
            executor.spin_once(timeout_sec=0.1)
    finally:
        executor.shutdown()

if __name__ == '__main__':
    main()
