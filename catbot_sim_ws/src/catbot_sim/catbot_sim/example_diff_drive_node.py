import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class DiffDriveTestNode(Node):
    def __init__(self):
        super().__init__('diff_drive_test_node')
        self.publisher = self.create_publisher(Twist, '/diff_drive_base_controller/cmd_vel_unstamped', 10)
        self.get_logger().info('Node created')

        self.command = Twist()
        self.command.linear.x = 0.1
        self.command.linear.y = 0.0
        self.command.linear.z = 0.0
        self.command.angular.x = 0.1
        self.command.angular.y = 0.0
        self.command.angular.z = 0.0

    def run(self):
        while rclpy.ok():
            self.publisher.publish(self.command)
            time.sleep(0.05)  # Sleep for 50 ms
            rclpy.spin_once(self)

def main(args=None):
    rclpy.init(args=args)
    node = DiffDriveTestNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
