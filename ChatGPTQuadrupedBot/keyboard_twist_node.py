import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import keyboard

class KeyboardTwistNode(Node):
    def __init__(self):
        super().__init__('keyboard_twist_node')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz
        self.get_logger().info("Keyboard Twist Node started. Use W, A, S, D to control.")

        # Initialize twist message
        self.twist = Twist()

    def timer_callback(self):
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0

        # Check for keyboard inputs
        if keyboard.is_pressed('w'):
            self.twist.linear.x = 1.0  # Move forward
        elif keyboard.is_pressed('s'):
            self.twist.linear.x = -1.0  # Move backward

        if keyboard.is_pressed('a'):
            self.twist.angular.z = 1.0  # Turn left
        elif keyboard.is_pressed('d'):
            self.twist.angular.z = -1.0  # Turn right

        self.publisher_.publish(self.twist)

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTwistNode()
    rclpy.spin(node)

    # Destroy the node explicitly
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
