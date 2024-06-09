#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/JointState.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include <tf/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "slam/process_sensor_data.h"
#include "slam/single_robot_mapping.h"

std::unique_ptr<lidar_data> lidarScan;
std::unique_ptr<robot_odometry> encoderValues;
ros::Publisher processed_data_pub;

void lidarCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    // Initialize lidarScan object using the first message
    if (!lidarScan)
    {
        lidarScan = std::make_unique<lidar_data>(
            msg->ranges.size(),
            value_range_t{.min{msg->range_min}, .max{msg->range_max}},
            value_range_t{.min{msg->angle_min}, .max{msg->angle_max}});
        ROS_INFO("Lidar Initialization Success!");
    }

    std::vector<float> scans{msg->ranges};
    sensor_msgs::LaserScan pub_msg;

    pub_msg.angle_increment = msg->angle_increment;
    pub_msg.angle_max = msg->angle_max;
    pub_msg.angle_min = msg->angle_min;
    pub_msg.range_max = msg->range_max;
    pub_msg.range_min = msg->range_min;
    pub_msg.ranges = lidarScan->m_process_data(scans);
    pub_msg.header.frame_id = "base_scan";
    pub_msg.intensities = msg->intensities;

    processed_data_pub.publish(pub_msg);
    std::cout << "Pub\n"
              << std::endl;
}

void encoderCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    // Initialize lidarScan object using the first message
    if (!encoderValues)
    {
        encoderValues = std::make_unique<robot_odometry>(
            4096,
            31.5,
            40);
        ROS_INFO("Encoder Initialization Success!");
    }

    auto quat = msg->pose.pose.orientation;
    tf2::Quaternion tf_quat(quat.x, quat.y, quat.z, quat.w);
    tf2::Matrix3x3 mat(tf_quat);
    double roll, pitch, yaw;
    mat.getRPY(roll, pitch, yaw);

    encoderValues->m_update_pose(
        msg->pose.pose.position.x,
        msg->pose.pose.position.y,
        yaw);

    pose_t currentPose = encoderValues->get_current_pose();

    std::cout << "X: " << currentPose.x << " | Y: " << currentPose.y << " | Theta: " << currentPose.theta << "\n"
              << std::endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "slam_integration");
    ros::NodeHandle nh;
    ros::Subscriber lidar_sub = nh.subscribe("scan", 1000, lidarCallback);
    ros::Subscriber encoder_sub = nh.subscribe("odom", 1000, encoderCallback);
    processed_data_pub = nh.advertise<sensor_msgs::LaserScan>("processed_lidar_data", 1000);

    ros::spin();

    return 0;
}
