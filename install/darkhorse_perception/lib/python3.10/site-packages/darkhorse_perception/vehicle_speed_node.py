#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import Jetson.GPIO as GPIO
import time, math

class VehicleSpeedNode(Node):
    def __init__(self):
        super().__init__('vehicle_speed_node')

        # -------- Parameters --------
        self.declare_parameter('hall1_pin', 17)
        self.declare_parameter('hall2_pin', 27)
        self.declare_parameter('hall3_pin', 22)
        self.declare_parameter('pole_pairs', 6)                    # motor-specific
        self.declare_parameter('gear_ratio', 9.0)                  # motor:wheel
        self.declare_parameter('wheel_diameter_m', 22.0*0.0254)    # 22" ≈ 0.5588 m
        self.declare_parameter('window_ms', 500)                   # update window
        self.declare_parameter('ema_alpha', 0.30)                  # smoothing [0..1]
        self.declare_parameter('transitions_per_e_rev', 6)         # 6 state changes / elec rev

        self.h1 = int(self.get_parameter('hall1_pin').value)
        self.h2 = int(self.get_parameter('hall2_pin').value)
        self.h3 = int(self.get_parameter('hall3_pin').value)
        self.pole_pairs = int(self.get_parameter('pole_pairs').value)
        self.gear = float(self.get_parameter('gear_ratio').value)
        self.wheel_diam = float(self.get_parameter('wheel_diameter_m').value)
        self.window_ms = int(self.get_parameter('window_ms').value)
        self.alpha = float(self.get_parameter('ema_alpha').value)
        self.trans_per_e = int(self.get_parameter('transitions_per_e_rev').value)

        self.circ = math.pi * self.wheel_diam

        # -------- GPIO --------
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.h1, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.h2, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.h3, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        self._transitions = 0
        self._last_state = 0
        self._mps_ema = 0.0

        # attach interrupts
        GPIO.add_event_detect(self.h1, GPIO.BOTH, callback=self._hall_isr)
        GPIO.add_event_detect(self.h2, GPIO.BOTH, callback=self._hall_isr)
        GPIO.add_event_detect(self.h3, GPIO.BOTH, callback=self._hall_isr)

        # -------- Publishers --------
        self.pub_mps   = self.create_publisher(Float32, 'VehicleSpeed', 10)
        self.pub_kmph  = self.create_publisher(Float32, 'VehicleSpeedKmph', 10)
        self.pub_mrpm  = self.create_publisher(Float32, 'MotorRPM', 10)
        self.pub_wrpm  = self.create_publisher(Float32, 'WheelRPM', 10)

        self.create_timer(self.window_ms/1000.0, self._tick)
        self.get_logger().info("✅ VehicleSpeedNode up (m/s + km/h + RPM). Note: Jetson internal pullups are ignored; use external pull-ups on Halls.")

    def _hall_isr(self, _channel):
        # Read all three Halls and form a 3-bit state; count state changes
        s1 = GPIO.input(self.h1)
        s2 = GPIO.input(self.h2)
        s3 = GPIO.input(self.h3)
        state = (s1 << 2) | (s2 << 1) | s3
        if state != self._last_state:
            self._transitions += 1
            self._last_state = state

    def _tick(self):
        # take and reset count
        trans = self._transitions
        self._transitions = 0

        # Electrical revolutions in the window:
        # For 3-hall BLDC you get 6 state changes per electrical revolution.
        e_rev = trans / max(1, self.trans_per_e)
        e_rpm = e_rev * (60000.0 / self.window_ms)

        # Mechanical motor RPM
        motor_rpm = e_rpm / max(1, self.pole_pairs)

        # Wheel RPM
        wheel_rpm = motor_rpm / max(1e-6, self.gear)

        # Speed: m/s (preferred for controllers), and km/h for UI
        mps = (wheel_rpm * self.circ) / 60.0
        self._mps_ema = (1.0 - self.alpha) * self._mps_ema + self.alpha * mps
        kmph = self._mps_ema * 3.6

        # Publish
        self.pub_mps.publish(Float32(data=float(self._mps_ema)))
        self.pub_kmph.publish(Float32(data=float(kmph)))
        self.pub_mrpm.publish(Float32(data=float(motor_rpm)))
        self.pub_wrpm.publish(Float32(data=float(wheel_rpm)))

        self.get_logger().info(f"MotorRPM={motor_rpm:.1f} | WheelRPM={wheel_rpm:.1f} | v={self._mps_ema:.2f} m/s ({kmph:.2f} km/h)")

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
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()

