#!/usr/bin/python3

import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import TransformStamped
import numpy as np
from tf_transformations import quaternion_from_euler

from vehicle_odom import Encoder, Steering

class StaticTFPublisher(Node):

    def __init__(self):
        super().__init__("StaticTFPublisher")

        self.msg_transform = TransformStamped()
        self.transform_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.timer = self.create_timer(0.1, self.publish_static_transforms)

        self.encoder_left = Encoder(self, "/autodrive/f1tenth_1/left_encoder")
        self.encoder_right = Encoder(self, "/autodrive/f1tenth_1/right_encoder")
        self.steering = Steering(self, "/autodrive/f1tenth_1/steering")

    def broadcast_transform(self, child_frame_id, parent_frame_id, translation, rotation):
        self.msg_transform.header.stamp = self.get_clock().now().to_msg()
        self.msg_transform.child_frame_id = child_frame_id
        self.msg_transform.transform.translation.x = translation[0]
        self.msg_transform.transform.translation.y = translation[1]
        self.msg_transform.transform.translation.z = translation[2]
        self.msg_transform.transform.rotation.x = rotation[0]
        self.msg_transform.transform.rotation.y = rotation[1]
        self.msg_transform.transform.rotation.z = rotation[2]
        self.msg_transform.transform.rotation.w = rotation[3]
        self.msg_transform.header.frame_id = parent_frame_id
        self.transform_broadcaster.sendTransform(self.msg_transform)

    def publish_static_transforms(self):

        if(not(self.encoder_left.is_ready() and self.encoder_right.is_ready() and self.steering.is_ready())):
            return

        encoder_angles = [self.encoder_left.get_latest()[0], self.encoder_right.get_latest()[0]]
        steering = self.steering.get_latest()[0]

        self.broadcast_transform("f1tenth_copy", "odom", np.asarray([0.0, 0.0, 0.0]), np.asarray([0.0, 0.0, 0.0, 1.0]))
        self.broadcast_transform("left_encoder_copy", "f1tenth_copy", np.asarray([0.0, 0.12, 0.0]), quaternion_from_euler(0.0, 120*encoder_angles[0]%6.283, 0.0))
        self.broadcast_transform("right_encoder_copy", "f1tenth_copy", np.asarray([0.0, -0.12, 0.0]), quaternion_from_euler(0.0, 120*encoder_angles[1]%6.283, 0.0))
        self.broadcast_transform("imu_copy", "f1tenth_copy", np.asarray([0.08, 0.0, 0.055]), np.asarray([0.0, 0.0, 0.0, 1.0]))
        self.broadcast_transform("lidar_copy", "f1tenth_copy", np.asarray([0.2733, 0.0, 0.096]), np.asarray([0.0, 0.0, 0.0, 1.0]))
        self.broadcast_transform("front_left_wheel_copy", "f1tenth_copy", np.asarray([0.33, 0.118, 0.0]), quaternion_from_euler(0.0, 0.0, np.arctan((2*0.141537*np.tan(steering))/(2*0.141537-2*0.0765*np.tan(steering)))))
        self.broadcast_transform("front_right_wheel_copy", "f1tenth_copy", np.asarray([0.33, -0.118, 0.0]), quaternion_from_euler(0.0, 0.0, np.arctan((2*0.141537*np.tan(steering))/(2*0.141537+2*0.0765*np.tan(steering)))))
        self.broadcast_transform("rear_left_wheel_copy", "f1tenth_copy", np.asarray([0.0, 0.118, 0.0]), quaternion_from_euler(0.0, encoder_angles[0]%6.283, 0.0))
        self.broadcast_transform("rear_right_wheel_copy", "f1tenth_copy", np.asarray([0.0, -0.118, 0.0]), quaternion_from_euler(0.0, encoder_angles[1]%6.283, 0.0))

def main():
    rclpy.init()

    node = StaticTFPublisher()
    rclpy.spin(node)
    
    rclpy.shutdown()

if __name__ == "__main__":
    main()