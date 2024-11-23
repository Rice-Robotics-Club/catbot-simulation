import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import math

class QuadrupedController(Node):
    def __init__(self):
        super().__init__('quadruped_controller')

        # Subscriptions and Publishers
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.twist_callback, 10)
        self.joint_publisher = self.create_publisher(Float64MultiArray, '/catbot_controller/commands', 10) # /catbot_controller/commands

        # Gait settings
        self.step_height = 0.10  # Max foot lift during step (meters)
        self.step_length = 0.10  # Step length in x-direction (meters)
        self.gait_phase = 0.0  # Phase offset between legs
        self.frequency = 1.0  # Hz

        # Initial parameters
        self.timer = self.create_timer(0.01, self.update_gait)  # 100 Hz control loop
        self.cmd_vel = Twist()

        self.get_logger().info("Quadruped Controller Node has started.")

    def twist_callback(self, msg):
        self.cmd_vel = msg

    def calculate_leg_positions(self, time):
        # Phases: FL=0, FR=0.5, BL=0.5, BR=0
        phases = [0.0, 0.5, 0.5, 0.0]  # FL, FR, BL, BR
        positions = []

        for i, phase in enumerate(phases):
            phase_time = (time * self.frequency + phase) % 1.0
            x = self.cmd_vel.linear.x * phase_time * self.step_length
            y = 0.0
            z = -self.step_height * math.cos(phase_time * 2 * math.pi) if phase_time < 0.5 else 0.0
            positions.append((x, y, z))
        
        return positions

    def calculate_joint_angles(self, foot_positions):
        joint_angles = []
        for x, y, z in foot_positions:
            # Example IK (simplified, adjust based on actual leg geometry)
            l1 = 0.134  # Hip to knee length
            l2 = 0.113  # Knee to foot length
            hip_angle = math.atan2(y, x)
            d = math.sqrt(x**2 + y**2 + z**2)
            knee_angle = math.acos((l1**2 + l2**2 - d**2) / (2 * l1 * l2))
            lower_leg_angle = -knee_angle / 2
            upper_leg_angle = knee_angle / 2

            joint_angles.append([hip_angle, upper_leg_angle, lower_leg_angle])
        
        return joint_angles

    def update_gait(self):
        time = self.get_clock().now().nanoseconds / 1e9
        foot_positions = self.calculate_leg_positions(time)
        joint_angles = self.calculate_joint_angles(foot_positions)

        # Publish joint commands
        msg = Float64MultiArray()
        msg.data = [angle for angles in joint_angles for angle in angles]  # Flatten list
        self.joint_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = QuadrupedController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
