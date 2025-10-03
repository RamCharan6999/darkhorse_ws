#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import Jetson.GPIO as GPIO
import time
import math

class VehicleSpeedNode(Node):
    def __init__(self):
        super().__init__('vehicle_speed_node')

        # --- Parameters ---
        self.declare_parameter('hall1_pin', 17)
        self.declare_parameter('hall2_pin', 27)
        self.declare_parameter('hall3_pin', 22)
        self.declare_parameter('pole_pairs', 6)
        self.declare_parameter('gear_ratio', 9.0)
        self.declare_parameter('wheel_diameter_m', 22.0 * 0.0254)
        self.declare_parameter('window_ms', 500)

        self.hall1Pin = self.get_parameter('hall1_pin').value
        self.hall2Pin = self.get_parameter('hall2_pin').value
        self.hall3Pin = self.get_parameter('hall3_pin').value
        self.polePairs = self.get_parameter('pole_pairs').value
        self.gearRatio = self.get_parameter('gear_ratio').value
        self.wheelDiameter_m = self.get_parameter('wheel_diameter_m').value
        self.windowMs = self.get_parameter('window_ms').value

        self.wheelCircumference = math.pi * self.wheelDiameter_m

        # --- GPIO setup ---
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.hall1Pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.hall2Pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.hall3Pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        self.hallTransitions = 0
        self.lastState = 0
        self.lastMillis = time.time() * 1000

        GPIO.add_event_detect(self.hall1Pin, GPIO.BOTH, callback=self.hallISR)
        GPIO.add_event_detect(self.hall2Pin, GPIO.BOTH, callback=self.hallISR)
        GPIO.add_event_detect(self.hall3Pin, GPIO.BOTH, callback=self.hallISR)

        # Publisher
        self.pub_speed = self.create_publisher(Float32, '/VehicleSpeed', 10)

        # Timer to compute every windowMs
        self.create_timer(self.windowMs / 1000.0, self.update_speed)

        self.get_logger().info("✅ VehicleSpeedNode started")

    def hallISR(self, channel):
        h1 = GPIO.input(self.hall1Pin)
        h2 = GPIO.input(self.hall2Pin)
        h3 = GPIO.input(self.hall3Pin)
        state = (h1 << 2) | (h2 << 1) | h3
        if state != self.lastState:
            self.hallTransitions += 1
            self.lastState = state

    def update_speed(self):
        now = time.time() * 1000
        trans = self.hallTransitions
        self.hallTransitions = 0

        # Each electrical cycle has 6 hall states → 6 transitions
        electricalRev = trans / 2
        electricalRPM = electricalRev * (60000.0 / self.windowMs)
        motorRPM = electricalRPM / self.polePairs

        wheelRPM = motorRPM / self.gearRatio
        vehicleSpeed = (wheelRPM * self.wheelCircumference * 60.0) / 1000.0

        msg = Float32()
        msg.data = float(vehicleSpeed)
        self.pub_speed.publish(msg)

        self.get_logger().info(f"Motor RPM={motorRPM:.1f} | Speed={vehicleSpeed:.2f} km/h")

def main(args=None):
    rclpy.init(args=args)
    node = VehicleSpeedNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        GPIO.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

