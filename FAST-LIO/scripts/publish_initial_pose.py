#!/usr/bin/env python3
# coding=utf8
from __future__ import print_function, division, absolute_import

import argparse
import math
import rospy
import tf.transformations
from geometry_msgs.msg import Pose, Point, Quaternion, PoseWithCovarianceStamped

if __name__ == '__main__':
    try:
        parser = argparse.ArgumentParser()
        parser.add_argument('x', type=float)
        parser.add_argument('y', type=float)
        parser.add_argument('z', type=float)
        parser.add_argument('yaw', type=float, help="Yaw angle in degrees")
        parser.add_argument('pitch', type=float, help="Pitch angle in degrees")
        parser.add_argument('roll', type=float, help="Roll angle in degrees")
        args, unknown = parser.parse_known_args()

        # 각도 단위를 도에서 라디안으로 변환
        yaw = math.radians(args.yaw)
        pitch = math.radians(args.pitch)
        roll = math.radians(args.roll)

        rospy.init_node('publish_initial_pose')
        pub_pose = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)

        # 변환을 적용하여 pose 생성
        quat = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        xyz = [args.x, args.y, args.z]

        initial_pose = PoseWithCovarianceStamped()
        initial_pose.pose.pose = Pose(Point(*xyz), Quaternion(*quat))
        initial_pose.header.stamp = rospy.Time().now()
        initial_pose.header.frame_id = 'map'
        rospy.sleep(1)
        rospy.loginfo('Initial Pose: {} {} {} {} {} {}'.format(
            args.x, args.y, args.z, args.yaw, args.pitch, args.roll))
        pub_pose.publish(initial_pose)
    except Exception as e:
        rospy.logerr('Error occurred: {}'.format(e))
