#!/usr/bin/env python3

import rospy
import math
from nav_msgs.msg import Odometry
from slam_msgs.msg import lidar_rotation
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler, euler_from_quaternion

class RotatingOdometryNode:
    def __init__(self):
        rospy.init_node('rotating_odometry')
        self.odom_sub = rospy.Subscriber('/localization', Odometry, self.odom_callback)
        self.rotation_sub = rospy.Subscriber('/lidar_rotation', lidar_rotation, self.rotation_callback)
        self.odom_pub = rospy.Publisher('/transformed_localization', Odometry, queue_size=10)
        self.current_rotation = 0.0

    def rotation_callback(self, msg):
        self.current_rotation = msg.rotation
        rospy.loginfo(f"Current rotation: {self.current_rotation}")

    def odom_callback(self, msg):
        transformed_odom = msg

        # Get current Quaternion
        q_orig = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )

        # Quaternion to roll, pitch, yaw
        roll, pitch, yaw = euler_from_quaternion(q_orig)

        # Adjust the pitch and yaw
        pitch += 45.0 * (math.pi / 180.0)  # Add 45 degrees to pitch
        yaw += self.current_rotation * (math.pi / 180.0)  # Add the current rotation to yaw

        # Convert back to Quaternion
        q_final = quaternion_from_euler(roll, pitch, yaw)

        # Apply the transformed quaternion to the message
        transformed_odom.pose.pose.orientation = Quaternion(*q_final)

        # Publish the transformed message
        self.odom_pub.publish(transformed_odom)

if __name__ == '__main__':
    try:
        node = RotatingOdometryNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
