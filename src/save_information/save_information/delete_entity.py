import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import DeleteEntity

class DeleteEntityClient(Node):
    def __init__(self):
        super().__init__('delete_entity_client')
        self.declare_parameter('entity_name', 'airplane_1')

        self.client = self.create_client(DeleteEntity, '/gazebo/delete_entity')

        timeout_sec = 5.0  # Wait for a few seconds before giving up
        if not self.client.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().error(f"Service /gazebo/delete_entity not available after {timeout_sec} seconds. Exiting.")
            rclpy.shutdown()
            exit(1)  # Exit the program immediately

        self.request = DeleteEntity.Request()
        self.request.name = self.get_parameter('entity_name').value
        self.future = self.client.call_async(self.request)

def main(args=None):
    rclpy.init(args=args)
    node = DeleteEntityClient()
    rclpy.spin_until_future_complete(node, node.future)

    if node.future.done() and node.future.result() is not None:
        node.get_logger().info(f"Deleted entity: {node.request.name}")
    else:
        node.get_logger().error("Failed to delete entity or service call failed.")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
