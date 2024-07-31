#include <iostream>
#include <condition_variable>
#include <omp.h>
#include <mutex>
#include <math.h>
#include <thread>
#include <csignal>
#include <unistd.h>
#include <ros/ros.h>
#include <Eigen/Core>
#include <visualization_msgs/Marker.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Vector3.h>
#include <slam_msgs/lidar_rotation.h>
#include <../include/rotating_imu.hpp>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Matrix3x3.h>

#define PI_M (3.14159265358)
#define grav (-9.81)

std::vector<double> extrinT;
double extrinT_arr[3] = {-0.09, 0.0, 0.1};
int rpm = 0;

using namespace std;

rotating_imu_node::rotating_imu_node(int argc, char **argv)
{
    ros::init(argc, argv, "rotating_imu");
    ros::NodeHandle nh;

    sub_imu = nh.subscribe("/imu/data_raw", 200000, &rotating_imu_node::imu_cbk, this);
    sub_rot = nh.subscribe("/lidar_rotation", 200000, &rotating_imu_node::lidar_rotation_cbk, this);
    pub_imu = nh.advertise<sensor_msgs::Imu>("/rotated_imu", 100000);

    nh.param<vector<double>>("mapping/extrinsic_T", extrinT, vector<double>());
    nh.param<int>("mapping/rpm", rpm, 5);

    ros::Rate rate(5000);
    bool status = ros::ok();

    cout << "Calc rpm: " << rpm << endl;

    for (int i = 0; i < 3; ++i)
    {
        extrinT_arr[i] = extrinT[i];
    }

    while (status)
    {
        ros::spinOnce();
        if (sync_packages(Sensors))
        {
            run();
        }
        status = ros::ok();
        rate.sleep();
    }
}

rotating_imu_node::~rotating_imu_node()
{
}

void rotating_imu_node::run()
{
    // getting sensor
    sensor_msgs::Imu imu_origin = *(Sensors.imu.back());
    // sensor_msgs::Imu imu_origin = *imu_buff;
    psi = -Sensors.rotation.back()->rotation;
    // psi = -(rotation_buff->rotation);

    // Degree to Radian
    double psi_radian = psi * M_PI / 180.0;     // yaw
    double theta_radian = theta * M_PI / 180.0; // pitch
    double fai_radian = fai * M_PI / 180.0;     // roll

    // RPM to rad/s
    rad_sec = -rpm * M_PI / 30;

    Eigen::Vector3d original_angular_vel(imu_origin.angular_velocity.x,
                                         imu_origin.angular_velocity.y,
                                         imu_origin.angular_velocity.z);

    Eigen::Matrix3d R_Matrix;
    Eigen::Vector3d transformed_angular_vel;

    if (psi == 1.0)
    {
        R_Matrix = make_R_matrix(0.0, theta_radian, 0.0);
        transformed_angular_vel = R_Matrix * original_angular_vel;
    }
    else
    {
        // Rotation Matrix (imu frame to lidar frame)
        R_Matrix = make_R_matrix(psi_radian, theta_radian, fai_radian);

        // Rotating angular velocity
        Eigen::Vector3d add_angular_vel(0.0, 0.0, rad_sec);
        // R_Matrix = make_R_matrix(psi_radian, theta_radian, fai_radian);
        add_angular_vel = R_Matrix * add_angular_vel;

        // 최종 변환 angular_velocity
        transformed_angular_vel = R_Matrix * original_angular_vel + add_angular_vel;
    }

    // Rotating linear acceleration
    Eigen::Vector3d original_linear_acc(imu_origin.linear_acceleration.x,
                                        imu_origin.linear_acceleration.y,
                                        imu_origin.linear_acceleration.z);

    Eigen::Vector3d transformed_linear_acc = R_Matrix * original_linear_acc;

    if (!(psi == 1.0))
    {
        // roll과 yaw에 의한 구심 가속도 제거
        Eigen::Vector3d transformed_distance(extrinT_arr[0], extrinT_arr[1], extrinT_arr[2]); // lidar sensing 중심과 imu 사이의 거리
        // transformed_distance = R_Matrix * transformed_distance;

        double centripetal_acc_x = pow(original_angular_vel[0], 2) * transformed_distance[0];
        double centripetal_acc_y = pow(original_angular_vel[1], 2) * transformed_distance[1];
        double centripetal_acc_z = pow(original_angular_vel[2], 2) * transformed_distance[2];

        Eigen::Vector3d centripetal_linear_acc(centripetal_acc_x, centripetal_acc_y, centripetal_acc_z);
        centripetal_linear_acc = R_Matrix * centripetal_linear_acc;

        // 가속도에서 구심가속도 제거
        transformed_linear_acc = transformed_linear_acc - centripetal_linear_acc;
    }

    // publishing imu msg
    sensor_msgs::Imu rotated_imu_msg;

    rotated_imu_msg.linear_acceleration.x = transformed_linear_acc[0];
    rotated_imu_msg.linear_acceleration.y = transformed_linear_acc[1];
    rotated_imu_msg.linear_acceleration.z = transformed_linear_acc[2];
    rotated_imu_msg.angular_velocity.x = transformed_angular_vel[0];
    rotated_imu_msg.angular_velocity.y = transformed_angular_vel[1];
    rotated_imu_msg.angular_velocity.z = transformed_angular_vel[2];
    rotated_imu_msg.header.stamp = imu_origin.header.stamp;

    pub_imu.publish(rotated_imu_msg);
}

Eigen::Matrix3d rotating_imu_node::make_R_matrix(double yaw_radian, double pitch_radian, double roll_radian)
{
    Eigen::Matrix3d yawMatrix;
    yawMatrix << cos(yaw_radian), sin(yaw_radian), 0,
        -sin(yaw_radian), cos(yaw_radian), 0,
        0, 0, 1;

    Eigen::Matrix3d pitchMatrix;
    pitchMatrix << cos(pitch_radian), 0, -sin(pitch_radian),
        0, 1, 0,
        sin(pitch_radian), 0, cos(pitch_radian);

    // Eigen::Matrix3d rollMatrix;
    // rollMatrix << 1, 0,            0,
    //               0, cos(roll_radian), -sin(roll_radian),
    //               0, sin(roll_radian),  cos(roll_radian);

    // Result_Matrix = yawMatrix * pitchMatrix * rollMatrix;

    Eigen::Matrix3d Result_Matrix = pitchMatrix * yawMatrix;

    return Result_Matrix;
}

void rotating_imu_node::imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in)
{
    sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in));

    msg->header.stamp = ros::Time().fromSec(msg_in->header.stamp.toSec());

    double timestamp = msg->header.stamp.toSec();

    mtx_buffer.lock();

    if (timestamp < last_timestamp_imu)
    {
        ROS_WARN("imu loop back, clear buffer");
        imu_buffer.clear();
    }

    last_timestamp_imu = timestamp;

    imu_buffer.push_back(msg);
    imu_buff = msg;
    mtx_buffer.unlock();
}

void rotating_imu_node::lidar_rotation_cbk(const slam_msgs::lidar_rotation::ConstPtr &msg_in)
{
    slam_msgs::lidar_rotation::Ptr msg(new slam_msgs::lidar_rotation(*msg_in));

    msg->header.stamp = ros::Time().fromSec(msg_in->header.stamp.toSec());

    double timestamp = msg->header.stamp.toSec();

    mtx_buffer.lock();

    if (timestamp < last_timestamp_lidar_rot)
    {
        ROS_WARN("lidar rotation loop back, clear buffer");
        rotation_buffer.clear();
    }

    last_timestamp_lidar_rot = timestamp;

    rotation_buffer.push_back(msg);
    rotation_buff = msg;
    mtx_buffer.unlock();
}

bool rotating_imu_node::sync_packages(SensorGroup &meas)
{
    if (imu_buffer.empty() || rotation_buffer.empty())
    {
        return false;
    }

    sync_time = ros::Time::now().toSec();

    double rotation_time = rotation_buffer.front()->header.stamp.toSec();
    meas.rotation.clear();
    while ((!rotation_buffer.empty()) && (rotation_time < sync_time))
    {
        rotation_time = rotation_buffer.front()->header.stamp.toSec();
        if (rotation_time > sync_time)
            break;
        meas.rotation.push_back(rotation_buffer.front());
        rotation_buffer.pop_front();
    }

    double imu_time = imu_buffer.front()->header.stamp.toSec();
    meas.imu.clear();
    while ((!imu_buffer.empty()) && (imu_time < sync_time))
    {
        imu_time = imu_buffer.front()->header.stamp.toSec();
        if (imu_time > sync_time)
            break;
        meas.imu.push_back(imu_buffer.front());
        imu_buffer.pop_front();
    }

    return true;
}

int main(int argc, char **argv)
{
    rotating_imu_node run(argc, argv);
}