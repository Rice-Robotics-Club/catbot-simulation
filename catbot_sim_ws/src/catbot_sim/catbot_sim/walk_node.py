import rclpy
from std_msgs.msg import Float64MultiArray
from rclpy.node import Node
import math
import typing
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray


class WalkNode(Node):
    def __init__(self):
        super().__init__("test_node")

        self.get_logger().info(f"initializing {self.get_name()}")

        # yaml params
        self.stepHeight: float = self.declare_parameter("stepHeight", 1.0).value
        self.stepLength: float = self.declare_parameter("stepLength", 1.0).value
        self.heightOffset: float = self.declare_parameter("heightOffset", 4.5).value
        period: float = self.declare_parameter("period", 5.0).value

        # pub / sub
        self.leg_positions = self.create_publisher(
            Float64MultiArray, "/leg_positions", 10
        )
        self.twist = self.create_subscription(Twist, '/cmd_vel', self.twist_callback, 10)

        interval = 0.01
        self.timer = self.create_timer(interval, self.timer_callback)
        self.leg_poses_msg = Float64MultiArray()
        self.leg_poses_msg.data = []
        self.angles = [
            0.0, 
            math.pi, 
            math.pi / 2,
            3 * math.pi / 2
        ] # radians
        
        self.angle = 0.0
        self.cmd_vel = Twist()
        self.delta = (math.tau * interval) / period

    def twist_callback(self, msg):
        self.cmd_vel = msg

    def timer_callback(self) -> None: # super inefficient rn but we'll fix that later
        positions = []
        for angle in self.angles:
            # increment angle
            positions += [
                0.7, # x --- 0.7 is the offset to make hip motors 100% straight since origin of point is base of hip servo
                -self.heightOffset + max(self.stepHeight * math.sin(angle), 0), # y
                self.stepLength * math.cos(angle), # z 
            ]            
    
        self.leg_poses_msg.data = positions
        self.leg_positions.publish(self.leg_poses_msg)
        # self.get_logger().info(f"input position: {positions}")
        
        # update new angle
        for j in range(0, 4):
            self.angles[j] += (self.delta * self.cmd_vel.linear.x)



def main(args=None) -> None:
    rclpy.init(args=args)
    node = WalkNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
