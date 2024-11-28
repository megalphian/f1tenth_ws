#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

class WallFollow(Node):

    "wall following for simulation environment"

    def __init__(self):
        super().__init__('wall_follow')
        lidar_scan_topic = '/autodrive/f1tenth_1/lidar'
        twist_cmd_topic = '/cmd_vel'

        #create a subscriber to the lidar scan
        self.lidar_sub = self.create_subscription(LaserScan, lidar_scan_topic, self.callback, 10)

        # #create a publisher to the steering command
        # self.steering_pub = self.create_publisher(Float32, steering_cmd_topic, 10)

        # #create a publisher to the throttle command
        # self.throttle_pub = self.create_publisher(Float32, throttle_cmd_topic, 10)

        self.twist_pub = self.create_publisher(Twist, twist_cmd_topic, 10)

        #pid gains
        self.kp = 2.0
        self.kd = 0.1
        self.ki = 0

        #history
        self.error = 0
        self.prev_error = 0
        self.error_integral = 0
        self.current_t = 0
        self.prev_t = 0

    def getrange(self, msg, theta):
        #angle must be within range
        assert theta >= msg.angle_min and theta <= msg.angle_max
        #index of closest angle
        index = int((theta - msg.angle_min) / msg.angle_increment)
        #check for out of bounds
        if np.isinf(msg.ranges[index]) or np.isnan(msg.ranges[index]) or msg.ranges[index] > msg.range_max:
            return msg.range_max
        else:
            return msg.ranges[index]
        
    
    def calculate_error(self, msg,desired, isRad = False , lookahead = 0.5):
        #angle must be within range
        angle1 = -50
        angle2 = -80
        theta = angle1 - angle2
        if not isRad:
            theta = np.radians(theta)
            angle1 = np.radians(angle1)
            angle2 = np.radians(angle2)

        assert theta >= msg.angle_min and theta <= msg.angle_max

        a = self.getrange(msg, angle1)
        b = self.getrange(msg, angle2)
        print("a: ", a, "b: ", b)
        alpha = np.arctan2(a*np.sin(theta), a*np.cos(theta) - b)
        dist_to_wall = b*np.cos(alpha)
        future_diviation = lookahead*np.sin(alpha)
        print("dist: " , dist_to_wall, future_diviation)
        error = desired - (dist_to_wall + future_diviation)
        print("error: ", error)

        self.prev_error = self.error
        self.error = error
        self.error_integral += error

        #calc time
        self.prev_t = self.current_t
        self.current_t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        
    
    def steering_pid(self):
        steering_angle = self.kp*self.error + self.kd*(self.error - self.prev_error)/(self.current_t - self.prev_t) + self.ki*self.error_integral
        return steering_angle



    def callback(self, msg):
        #calculate error
        self.calculate_error(msg, 0.6)
         #calculate pid
        steering = self.steering_pid()
        throttle = 1.0
        
        #publish
        twist = Twist()
        twist.linear.x = throttle
        twist.angular.z = steering
        self.twist_pub.publish(twist)
 
def main():
	rclpy.init()
	node = WallFollow()
	rclpy.spin(node)
	rclpy.shutdown()
 
if __name__ == "__main__":
	main()