#!/usr/bin/python3

from math import atan

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

class VehicleController(Node):

    def __init__(self):
        super().__init__("VehicleController")
        
        self.throttle_publisher = self.create_publisher(Float32, "/autodrive/f1tenth_1/throttle_command", 10)
        self.steering_publisher = self.create_publisher(Float32, "/autodrive/f1tenth_1/steering_command", 10)

        self.vehicle_command_stream = self.create_subscription(Twist, "/cmd_vel", self.set_vehicle_commands, 10)

    def set_vehicle_commands(self, msg : Twist):
        throttle_command = Float32()
        steering_command = Float32()

        _throttle = msg.linear.x/(22.8)

        # Clamp Output
        if (_throttle < 0.0):
            if (_throttle < -1.0):
                _throttle = -1.0
        else:
            if (_throttle > 1.0):
                _throttle = 1.0

        throttle_command.data = _throttle

        if (msg.linear.x == 0.0):
            _steering_angle = 0.0
        else:
            _steering_angle = atan(0.3240*(msg.angular.z/msg.linear.x))
            _steering_angle = _steering_angle/0.5236
            if (_steering_angle < 0.0):
                if (_steering_angle < -0.5236):
                    _steering_angle = -0.5236
            else:
                if (_steering_angle > 0.5236):
                    _steering_angle = 0.5236

        steering_command.data = _steering_angle

        self.throttle_publisher.publish(throttle_command)
        self.steering_publisher.publish(steering_command)

def main():
    rclpy.init()

    node = VehicleController()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()