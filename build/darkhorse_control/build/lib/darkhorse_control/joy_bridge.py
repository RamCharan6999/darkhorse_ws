import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

try:
    from vehicle_msgs.msg import ControlCmd  # if custom message exists
except ImportError:
    from std_msgs.msg import Float32  # fallback for testing

class JoyBridge(Node):
    def __init__(self):
        super().__init__('joy_bridge')
        self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 10)

        # Publisher
        try:
            self.pub = self.create_publisher(ControlCmd, '/vehicle_control', 10)
            self.use_custom = True
        except Exception:
            self.pub_throttle = self.create_publisher(Float32, '/throttle_pct', 10)
            self.pub_brake = self.create_publisher(Float32, '/brake_pct', 10)
            self.pub_steering = self.create_publisher(Float32, '/steering_pct', 10)
            self.use_custom = False

    def cmd_cb(self, msg: Twist):
        if self.use_custom:
            cmd = ControlCmd()
            # map joystick linear.x → throttle/brake
            if msg.linear.x >= 0:
                cmd.throttle_pct = min(msg.linear.x * 100.0, 100.0)
                cmd.brake_pct = 0.0
            else:
                cmd.throttle_pct = 0.0
                cmd.brake_pct = min(-msg.linear.x * 100.0, 100.0)
            # map joystick angular.z → steering %
            cmd.steering_pct = (msg.angular.z + 1.0) * 50.0
            self.pub.publish(cmd)
            self.get_logger().info(f"Published vehicle_control: T={cmd.throttle_pct:.1f}, B={cmd.brake_pct:.1f}, S={cmd.steering_pct:.1f}")
        else:
            # fallback publishers
            throttle = max(0.0, msg.linear.x) * 100.0
            brake = max(0.0, -msg.linear.x) * 100.0
            steering = (msg.angular.z + 1.0) * 50.0
            self.pub_throttle.publish(Float32(data=throttle))
            self.pub_brake.publish(Float32(data=brake))
            self.pub_steering.publish(Float32(data=steering))
            self.get_logger().info(f"Published fallback: T={throttle:.1f}, B={brake:.1f}, S={steering:.1f}")

def main(args=None):
    rclpy.init(args=args)
    node = JoyBridge()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
