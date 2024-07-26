#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <boost/bind.hpp>

#include "sensor_msgs/LaserScan.h"
#include "slam/process_sensor_data.h"
#include "slam/single_robot_mapping.h"
#include "slam/visulaization.h"
///////////////////// Robot Specific Variables /////////////////////
/// Robot Dimensions
constexpr float robot_width = 160; // mm
/// LIDAR Specific Variables
constexpr std::size_t lidar_data_points_counts = 360;
const pose_t lidar_to_robot_transformation{0.0F, 0.0F, 0.0F};
/////////////////////////// Mapping Variables /////////////////////////////
constexpr float grid_size = 10.0f;

void lidarCallback(const sensor_msgs::LaserScan::ConstPtr &msg,
                   std::unique_ptr<robot_odometry> &robotOdomObject,
                   std::unique_ptr<lidar_data> &lidarObject,
                   std::unique_ptr<map_environment> &robotMap,
                   pose_t &current_pose,
                   std::vector<pose_t> &all_pose_robot_global_frame,
                   cv::Mat &slam_map_image)
{
    if (!lidarObject)
    {
        // Initialize lidarScan object using the first message
        if (!robotOdomObject)
        {
            return;
        }
        lidarObject = std::make_unique<lidar_data>(
            msg->ranges.size(),
            value_range_t{.min = msg->range_min, .max = msg->range_max},
            0.650F,
            lidar_to_robot_transformation,
            0.0F,
            1000.0F);
        robotMap = std::make_unique<map_environment>(1000.0, grid_size);
        ROS_INFO("Lidar Initialization Success!");
    }

    std::vector<float> scans{msg->ranges};
    const auto obstacles_robot_frame = lidarObject->process_lidar_scan_to_robot_frame(scans, current_pose);
    auto currentMap_robot_origin_frame = robotMap->updateMap_robot_origin_frame(obstacles_robot_frame, current_pose);

    all_pose_robot_global_frame.push_back(current_pose);

    Viz::gridToImage(slam_map_image, currentMap_robot_origin_frame);
    Viz::plotGraphPoints(slam_map_image, all_pose_robot_global_frame, Viz::red, slam_map_image.cols, robotMap);
    cv::imshow("Lidar Map", slam_map_image);
    cv::waitKey(10);
}

void encoderCallback(const nav_msgs::Odometry::ConstPtr &msg, // Position is in Meters
                     pose_t &currentPose,
                     std::unique_ptr<robot_odometry> &robotOdomObject)
{
    auto quat = msg->pose.pose.orientation;
    tf2::Quaternion tf_quat(quat.x, quat.y, quat.z, quat.w);
    tf2::Matrix3x3 mat(tf_quat);
    double roll, pitch, yaw;
    mat.getRPY(roll, pitch, yaw);

    // ROS_INFO("Encoder Callback");
    // Initialize lidarScan object using the first message
    if (!robotOdomObject)
    {
        robotOdomObject = std::make_unique<robot_odometry>(
            robot_width,
            msg->pose.pose.position.x * 1000.0F,
            msg->pose.pose.position.y * 1000.0F,
            yaw);
        ROS_INFO("Encoder Initialization Success!");
    }

    currentPose = robotOdomObject->m_update_pose(
        msg->pose.pose.position.x * 1000.0F,
        msg->pose.pose.position.y * 1000.0F,
        yaw);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "slam_integration");
    ros::NodeHandle nh;
    std::unique_ptr<lidar_data> lidarObject;
    std::unique_ptr<robot_odometry> robotOdomObject;
    std::unique_ptr<map_environment> robotMap;
    pose_t robot_pose;
    std::vector<pose_t> all_pose_robot_global_frame;
    cv::Mat slam_map_image;
    cv::namedWindow("Lidar Map", cv::WINDOW_AUTOSIZE);

    ros::Subscriber encoder_sub = nh.subscribe<nav_msgs::Odometry>("odom", 1000,
                                                                   boost::bind(
                                                                       encoderCallback,
                                                                       _1,
                                                                       boost::ref(robot_pose),
                                                                       boost::ref(robotOdomObject)));
    ros::Subscriber lidar_sub = nh.subscribe<sensor_msgs::LaserScan>("scan", 1000,
                                                                     boost::bind(
                                                                         lidarCallback,
                                                                         _1,
                                                                         boost::ref(robotOdomObject),
                                                                         boost::ref(lidarObject),
                                                                         boost::ref(robotMap),
                                                                         boost::ref(robot_pose),
                                                                         boost::ref(all_pose_robot_global_frame),
                                                                         boost::ref(slam_map_image)));
    ros::spin();
    cv::destroyWindow("Lidar Map");

    return 0;
}
