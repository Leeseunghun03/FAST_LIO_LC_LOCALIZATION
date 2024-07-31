#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import csv
import os
import math
import tf.transformations

class PathRecorder:
    def __init__(self):
        rospy.init_node('save_path', anonymous=True)

        home_dir = os.path.expanduser('~')
        self.file_path = os.path.join(home_dir, 'catkin_ws', 'maps', 'path.csv')
        os.makedirs(os.path.dirname(self.file_path), exist_ok=True)

        # Open the file in write mode ('w'), which clears existing content
        self.file = open(self.file_path, mode='w', newline='')
        self.writer = csv.writer(self.file)

        # Write header
        self.writer.writerow(['x', 'y', 'z', 'orientation_x', 'orientation_y', 'orientation_z', 'orientation_w'])

        self.path_sub = rospy.Subscriber('/path_updated', Path, self.path_callback)
        self.transformed_path_pub = rospy.Publisher('/transformed_path', Path, queue_size=10)

        self.pose_index = 0  # 현재 처리된 포즈의 인덱스

    def path_callback(self, msg):
        transformed_path = Path()
        transformed_path.header = msg.header

        # Process only if there are poses in the message
        if msg.poses and self.pose_index < len(msg.poses):
            pose = msg.poses[self.pose_index]
            position = pose.pose.position
            orientation = pose.pose.orientation

            # Convert quaternion to matrix
            quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
            matrix = tf.transformations.quaternion_matrix(quaternion)

            pitch_angle = -45.0 * (math.pi / 180.0)
            pitch_matrix = tf.transformations.rotation_matrix(pitch_angle, [0, 1, 0])

            # Apply pitch rotation
            rotated_matrix = tf.transformations.concatenate_matrices(pitch_matrix, matrix)

            # Apply rotation to position
            original_position = [position.x, position.y, position.z, 1.0]
            rotated_position = tf.transformations.translation_from_matrix(
                tf.transformations.concatenate_matrices(
                    pitch_matrix,
                    tf.transformations.translation_matrix(original_position)
                )
            )

            # Convert rotated matrix back to quaternion
            rotated_quaternion = tf.transformations.quaternion_from_matrix(rotated_matrix)

            # Create transformed pose
            transformed_pose = PoseStamped()
            transformed_pose.header = pose.header
            transformed_pose.pose.position.x = rotated_position[0]
            transformed_pose.pose.position.y = rotated_position[1]
            transformed_pose.pose.position.z = rotated_position[2]
            transformed_pose.pose.orientation.x = rotated_quaternion[0]
            transformed_pose.pose.orientation.y = rotated_quaternion[1]
            transformed_pose.pose.orientation.z = rotated_quaternion[2]
            transformed_pose.pose.orientation.w = rotated_quaternion[3]

            # Write to CSV file
            self.writer.writerow([rotated_position[0], rotated_position[1], rotated_position[2], 
                                  rotated_quaternion[0], rotated_quaternion[1], rotated_quaternion[2], rotated_quaternion[3]])

            # Increment pose index for the next callback
            self.pose_index += 1

            # Store the latest transformed pose for publishing later if needed
            self.latest_transformed_pose = transformed_pose

        # Publish the transformed path (empty in this case, since we only publish the last pose)
        self.transformed_path_pub.publish(transformed_path)

    def run(self):
        rospy.spin()

        # Close the file when shutting down
        if self.file is not None:
            self.file.close()

if __name__ == '__main__':
    recorder = PathRecorder()
    recorder.run()
