#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>

ros::Publisher rotated_path_pub;

void pathCallback(const nav_msgs::Path::ConstPtr& msg) {
    nav_msgs::Path rotated_path = *msg;
    rotated_path.header.stamp = ros::Time::now();

    tf2::Quaternion rotation_quaternion;
    rotation_quaternion.setRPY(0, M_PI / 4, 0); // pitch 45 rotate

    tf2::Transform rotation_transform(rotation_quaternion);

    for (auto& pose : rotated_path.poses) {
        geometry_msgs::PoseStamped& pose_stamped = pose;
        tf2::Vector3 position(pose_stamped.pose.position.x,
                              pose_stamped.pose.position.y,
                              pose_stamped.pose.position.z);
        tf2::Quaternion orientation;
        tf2::fromMsg(pose_stamped.pose.orientation, orientation);

        tf2::Transform pose_transform(orientation, position);

        tf2::Transform rotated_pose_transform = rotation_transform * pose_transform;

        tf2::Vector3 rotated_position = rotated_pose_transform.getOrigin();
        tf2::Quaternion rotated_orientation = tf2::Quaternion(rotated_pose_transform.getRotation());

        pose_stamped.pose.position.x = rotated_position.x();
        pose_stamped.pose.position.y = rotated_position.y();
        pose_stamped.pose.position.z = rotated_position.z();
        pose_stamped.pose.orientation = tf2::toMsg(rotated_orientation);
    }

    rotated_path_pub.publish(rotated_path);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "rotating_path");
    ros::NodeHandle nh;

    ros::Subscriber path_sub = nh.subscribe("path_updated", 10, pathCallback);
    rotated_path_pub = nh.advertise<nav_msgs::Path>("trajectory", 10);

    ros::spin();
    return 0;
}
