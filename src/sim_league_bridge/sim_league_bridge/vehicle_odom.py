#!/usr/bin/python3

from collections import deque
from dataclasses import dataclass
from math import tan, isinf, sin, cos

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32

from tf_transformations import quaternion_from_euler

class Encoder:

    def __init__(self, node, topic : str):
        self.sub = node.create_subscription(JointState, topic, self.read_encoder, 20)
        self.valuedq = deque(maxlen=1000)
        self.timedq = deque(maxlen=1000)
        self.msg_count = 0

    def read_encoder(self, msg : JointState):
        self.valuedq.append(msg.position[0])
        self.timedq.append(msg.header.stamp)
        self.msg_count += 1

    def get_latest(self):
        return self.valuedq[-1], self.timedq[-1]

    def get_buffer(self):
        return self.valuedq, self.timedq

    def get_latest_velocity(self):
        dt = (self.timedq[-1].sec + 1e-9*self.timedq[-1].nanosec) - (self.timedq[-2].sec + 1e-9*self.timedq[-2].nanosec)
        return (self.valuedq[-1]-self.valuedq[-2])/(dt*7.5)

    def is_ready(self):
        return self.msg_count > 2

class Steering:

    def __init__(self, node, topic : str):
        self.node = node
        self.msg_count = 0; self.steering_angle = 0
        self.msg_time = None
        self.steering = node.create_subscription(Float32, topic, self.steering_callback, 20)

    def steering_callback(self, msg : Float32):
        self.steering_angle = msg.data
        secnsec = self.node.get_clock().now().seconds_nanoseconds()
        self.msg_time = secnsec[0] + 1e-9*secnsec[1]
        self.msg_count += 1
    
    def get_latest(self):
        return self.steering_angle, self.msg_time

    def is_ready(self):
        return self.msg_count > 2

@dataclass
class VehicleState:
    x : float
    y : float
    heading : float

    linear_velocity : float
    steering_angle : float

@dataclass
class VehicleParameters:
    wheelbase : float
    track : float
    wheel_radius : float

class VehicleOdometry(Node):

    def __init__(self):
        super().__init__("VehicleOdometry")
        self.left_encoder = Encoder(self, "/autodrive/f1tenth_1/left_encoder")
        self.right_encoder = Encoder(self, "/autodrive/f1tenth_1/right_encoder")

        self.steering = Steering(self, "/autodrive/f1tenth_1/steering")

        self.publisher = self.create_publisher(Odometry, "/wheel_odom", 100)
        self.timer = self.create_timer(0.01, self.timer_callback)

        # Odom always considers itself to start from origin.
        self.internal_state = VehicleState(0.0, 0.0, 0.0, 0.0, 0.0)

        # https://autodrive-ecosystem.github.io/competitions/f1tenth-sim-racing-guide/
        self.vehicle = VehicleParameters(0.3240, 0.2360, 0.0590)

        self.prev_time = self.get_clock().now().seconds_nanoseconds()

    # https://github.com/ros-controls/ros2_controllers/blob/9b344c7b5c06879b85cd4bcc135e35b5f7eedc2f/steering_controllers_library/src/steering_odometry.cpp#L131
    def timer_callback(self):
        if not self.left_encoder.is_ready() or not self.right_encoder.is_ready() or not self.steering.is_ready():
            return

        msg = Odometry()

        # First calculate how much the encoder changed
        left_wheel_traction_velocity = self.left_encoder.get_latest_velocity()
        right_wheel_traction_velocity = self.right_encoder.get_latest_velocity()

        # Then calculate the steering angle
        steering_angle, msg_time = self.steering.get_latest()

        # Next Calculate how the odometry has to be shifted based on this model.
        left_wheel_velocity = left_wheel_traction_velocity * self.vehicle.wheel_radius
        right_wheel_velocity = right_wheel_traction_velocity * self.vehicle.wheel_radius

        if (tan(steering_angle) == 0):
            linear_velocity = (left_wheel_velocity + right_wheel_velocity) * 0.5
        else:
            turning_radius = self.vehicle.wheelbase/tan(steering_angle)
            lvel = left_wheel_velocity * turning_radius / (turning_radius + self.vehicle.track * 0.5)
            rvel = right_wheel_velocity * turning_radius / (turning_radius - self.vehicle.track * 0.5)
            linear_velocity = (lvel + rvel)*0.5

        angular_velocity = tan(steering_angle) * linear_velocity / self.vehicle.wheelbase

        now = self.get_clock().now().seconds_nanoseconds()
        dt = (now[0] + 1e-9*now[1]) - (self.prev_time[0] + 1e-9*self.prev_time[1])
        self.internal_state.heading += angular_velocity*dt
    
        self.internal_state.x += linear_velocity*cos(self.internal_state.heading)*dt
        self.internal_state.y += linear_velocity*sin(self.internal_state.heading)*dt

        self.internal_state.linear_velocity = linear_velocity
        self.internal_state.steering_angle = steering_angle

        msg.header.frame_id = "world"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.child_frame_id = "f1tenth_1"
        msg.pose.pose.position.x = self.internal_state.x
        msg.pose.pose.position.y = self.internal_state.y
        
        quat = quaternion_from_euler(0.0, 0.0, self.internal_state.heading)
        msg.pose.pose.orientation.x = quat[0]; msg.pose.pose.orientation.z = quat[2]
        msg.pose.pose.orientation.y = quat[1]; msg.pose.pose.orientation.w = quat[3]

        msg.pose.covariance = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                               0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 
                               0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 
                               0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 
                               0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 
                               0.0, 0.0, 0.0, 0.0, 0.0, 1.0]

        msg.twist.twist.linear.x = linear_velocity
        msg.twist.twist.angular.z = angular_velocity

        self.publisher.publish(msg)

        self.prev_time = now

def main():
    rclpy.init()

    node = VehicleOdometry()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()