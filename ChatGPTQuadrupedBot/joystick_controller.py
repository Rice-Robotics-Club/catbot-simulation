import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class JoystickController(Node):
    def __init__(self):
        super().__init__('joystick_controller')
        self.subscription = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info("Joystick Controller Node has started.")

    def joy_callback(self, msg):
        twist = Twist()
        twist.linear.x = msg.axes[1]  # Forward/Backward
        twist.angular.z = msg.axes[0]  # Left/Right
        self.publisher.publish(twist)
        self.get_logger().info(f"Published Twist: linear.x={twist.linear.x}, angular.z={twist.angular.z}")

def main(args=None):
    rclpy.init(args=args)
    node = JoystickController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
