import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class FakeSpeedPublisher(Node):
    def __init__(self):
        super().__init__('fake_speed_publisher')
        self.publisher_ = self.create_publisher(Float32, '/VehicleSpeed', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info('✅ Fake Speed Publisher started, publishing 1.0 m/s.')

    def timer_callback(self):
        msg = Float32()
        msg.data = 1.0  # Constant speed of 1.0 m/s
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = FakeSpeedPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
