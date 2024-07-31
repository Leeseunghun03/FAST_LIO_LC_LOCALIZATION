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
#include <geometry_msgs/Quaternion.h>
#include <slam_msgs/lidar_rotation.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Matrix3x3.h>

class rotating_imu_node
{
public:
    rotating_imu_node(int argc, char **argv);
    ~rotating_imu_node();

private:
    std::deque<sensor_msgs::Imu::Ptr> imu_buffer;
    std::deque<slam_msgs::lidar_rotation::Ptr> rotation_buffer;

    struct SensorGroup
    {
        std::deque<sensor_msgs::Imu::Ptr> imu;
        std::deque<slam_msgs::lidar_rotation::Ptr> rotation;
    };

    sensor_msgs::Imu::Ptr imu_buff;
    slam_msgs::lidar_rotation::Ptr rotation_buff;

    SensorGroup Sensors;

    std::mutex mtx_buffer;

    double rad_sec = 0.0;
    double last_timestamp_imu = -1.0, last_timestamp_lidar_rot = -1.0, sync_time = -1.0;
    double theta = -45.0, fai = 0.0, psi = 0.0;

    Eigen::Matrix3d R_Matrix;

    void imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in);
    void lidar_rotation_cbk(const slam_msgs::lidar_rotation::ConstPtr &msg_in);
    bool sync_packages(SensorGroup &meas);
    Eigen::Matrix3d make_R_matrix(double yaw_radian, double pitch_radian, double roll_radian);
    void run();

    ros::Subscriber sub_imu;
    ros::Subscriber sub_rot;

    ros::Publisher pub_imu;
};