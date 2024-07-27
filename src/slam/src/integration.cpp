#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "std_msgs/Bool.h"

#include <boost/bind.hpp>
#include <opencv2/opencv.hpp>
#include <thread>
#include <mutex>

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
////////////////////////// Thread Specific Variables ////////////////////////
#define NUM_ROBOTS 3

typedef struct
{
    std::unique_ptr<lidar_data> lidarObject;
    std::unique_ptr<robot_odometry> robotOdomObject;
    std::unique_ptr<map_environment> robotMap;
    pose_t robot_pose;
    std::vector<pose_t> all_pose_robot_global_frame;
    cv::Mat slam_map_image;
    ros::Subscriber encoder_sub;
    ros::Subscriber lidar_sub;
    std::mutex data_mutex; // Mutex to protect data access
    ros::Publisher motion_pub;

} Robot;

void lidarCallback(const sensor_msgs::LaserScan::ConstPtr &msg,
                   Robot &robot,
                   const int idx)
{
    std::lock_guard<std::mutex> lock(robot.data_mutex);

    if (!robot.lidarObject)
    {
        // Initialize lidarScan object using the first message
        if (!robot.robotOdomObject)
        {
            return;
        }
        robot.lidarObject = std::make_unique<lidar_data>(
            msg->ranges.size(),
            value_range_t{.min = msg->range_min, .max = msg->range_max},
            0.650F,
            lidar_to_robot_transformation,
            0.0F,
            1000.0F);
        robot.robotMap = std::make_unique<map_environment>(1000.0, grid_size);
        ROS_INFO("Lidar Initialization Success!");
    }

    std::vector<float> scans{msg->ranges};
    const auto obstacles_robot_frame = robot.lidarObject->process_lidar_scan_to_robot_frame(scans, robot.robot_pose);
    auto currentMap_robot_origin_frame = robot.robotMap->updateMap_robot_origin_frame(obstacles_robot_frame, robot.robot_pose);

    robot.all_pose_robot_global_frame.push_back(robot.robot_pose);

    Viz::gridToImage(robot.slam_map_image, currentMap_robot_origin_frame);
    Viz::plotGraphPoints(robot.slam_map_image, robot.all_pose_robot_global_frame, Viz::red, robot.slam_map_image.cols, robot.robotMap);
}

void encoderCallback(const nav_msgs::Odometry::ConstPtr &msg, // Position is in Meters
                     Robot &robot)
{
    std::lock_guard<std::mutex> lock(robot.data_mutex);

    auto quat = msg->pose.pose.orientation;
    tf2::Quaternion tf_quat(quat.x, quat.y, quat.z, quat.w);
    tf2::Matrix3x3 mat(tf_quat);
    double roll, pitch, yaw;
    mat.getRPY(roll, pitch, yaw);

    // ROS_INFO("Encoder Callback");
    // Initialize lidarScan object using the first message
    if (!robot.robotOdomObject)
    {
        robot.robotOdomObject = std::make_unique<robot_odometry>(
            robot_width,
            msg->pose.pose.position.x * 1000.0F,
            msg->pose.pose.position.y * 1000.0F,
            yaw);
        ROS_INFO("Encoder Initialization Success!");
        std_msgs::Bool motion_msg;
        motion_msg.data = true; // or false based on your control logic
        robot.motion_pub.publish(motion_msg);
    }

    robot.robot_pose = robot.robotOdomObject->m_update_pose(
        msg->pose.pose.position.x * 1000.0F,
        msg->pose.pose.position.y * 1000.0F,
        yaw);
}

void displayThread(std::vector<std::unique_ptr<Robot>> &robots)
{
    while (ros::ok())
    {
        for (int i = 0; i < NUM_ROBOTS; ++i)
        {
            std::lock_guard<std::mutex> lock(robots[i]->data_mutex);

            if (!robots[i]->slam_map_image.empty())
                cv::imshow("Lidar Map " + std::to_string(i), robots[i]->slam_map_image);
        }
        cv::waitKey(1); // Adjust the delay as needed
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "slam_integration");
    ros::NodeHandle nh;

    std::vector<std::thread> robot_threads;
    std::vector<std::unique_ptr<Robot>> robots;

    for (int i = 0; i < NUM_ROBOTS; ++i)
    {
        robots.push_back(std::make_unique<Robot>());
    }

    for (int i = 0; i < NUM_ROBOTS; ++i)
    {
        robot_threads.emplace_back([&nh, &robots, i]()
                                   {
            auto &robotData = *robots[i];
            robotData.encoder_sub = nh.subscribe<nav_msgs::Odometry>( "tb3_"+std::to_string(i)+"/odom", 1000,
                                                                      boost::bind(encoderCallback, _1, boost::ref(robotData)));
            robotData.lidar_sub = nh.subscribe<sensor_msgs::LaserScan>("tb3_"+std::to_string(i)+"/scan", 1000,
                                                                       boost::bind(lidarCallback, _1, boost::ref(robotData), i));
            robotData.motion_pub = nh.advertise<std_msgs::Bool>("tb3_" + std::to_string(i) + "/motion_control", 10);

            ros::MultiThreadedSpinner spinner(4); // Use a multi-threaded spinner
            spinner.spin(); });
    }

    std::thread display_thread(displayThread, std::ref(robots));

    for (auto &thread : robot_threads)
    {
        if (thread.joinable())
        {
            thread.join();
        }
    }

    if (display_thread.joinable())
    {
        display_thread.join();
    }

    return 0;
}
