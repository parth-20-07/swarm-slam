// ROS Libraries
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "std_msgs/Bool.h"

// CPP Libraries
#include <boost/bind.hpp>
#include <opencv2/opencv.hpp>
#include <thread>
#include <mutex>
#include <random>
#include <chrono>

// Custom headers
#include "sensor_msgs/LaserScan.h"
#include "slam/process_sensor_data.h"
#include "slam/single_robot_mapping.h"
#include "slam/visulaization.h"
#include "slam/map_merging.h"
///////////////////// Robot Specific Variables /////////////////////
/// Robot Dimensions
constexpr float robot_width = 160; // mm
/// LIDAR Specific Variables
constexpr std::size_t lidar_data_points_counts = 360;
const pose_t lidar_to_robot_transformation{0.0F, 0.0F, 0.0F};
/////////////////////////// Mapping Variables /////////////////////////////
constexpr float grid_size = 20.0f;
////////////////////////// Thread Specific Variables ////////////////////////
#define NUM_ROBOTS 3
#define MERGING_TIME_SEC 20

ros::NodeHandle *nh = nullptr; // Define the global pointer

typedef struct
{
    std::string robotTag;
    int robotID;

    std::unique_ptr<lidar_data> lidarObject;
    std::unique_ptr<robot_odometry> robotOdomObject;
    std::unique_ptr<map_environment> robotMap;

    pose_t robot_pose;
    pose_t global_pose;

    ros::Subscriber encoder_sub;
    ros::Subscriber lidar_sub;
    ros::Publisher motion_pub;

    std::mutex data_mutex; // Mutex to protect data access

    cv::Mat slam_map_image;
    bool motion_enabled = true;
    std::vector<pose_t> all_pose_robot_global_frame;

} Robot;

void encoderCallback(const nav_msgs::Odometry::ConstPtr &msg, // Position is in Meters
                     Robot &robot);
void lidarCallback(const sensor_msgs::LaserScan::ConstPtr &msg,
                   Robot &robot,
                   const int idx);

void setRobotMotion(Robot &robot, bool enable, ros::NodeHandle &nh)
{
    std_msgs::Bool motion_msg;
    motion_msg.data = enable;
    robot.motion_pub.publish(motion_msg);
    robot.motion_enabled = enable;

    if (enable)
    {
        robot.encoder_sub = nh.subscribe<nav_msgs::Odometry>(robot.robotTag + "/odom", 1000,
                                                             boost::bind(encoderCallback, _1, boost::ref(robot)));
        robot.lidar_sub = nh.subscribe<sensor_msgs::LaserScan>(robot.robotTag + "/scan", 1000,
                                                               boost::bind(lidarCallback, _1, boost::ref(robot), robot.robotID));
    }
    else
    {
        robot.lidar_sub.shutdown();
        robot.encoder_sub.shutdown();
    }
}

#include <random>
#include <mutex>
#include <memory>
#include <vector>

void randomRobotInteraction(std::vector<std::unique_ptr<Robot>> &robots)
{

    static std::random_device rd; // Used to obtain a seed for the random number engine
    static std::mt19937 g(rd());  // Standard mersenne_twister_engine seeded with rd()
    std::uniform_int_distribution<unsigned int> dist(0, UINT_MAX);

    int idx1 = dist(g) % NUM_ROBOTS;
    int idx2 = dist(g) % NUM_ROBOTS;
    while (idx1 == idx2)
    { // Ensure two different indices
        idx2 = dist(g) % NUM_ROBOTS;
    }

    int lower = std::min(idx1, idx2);
    int higher = std::max(idx1, idx2);
    ROS_INFO("Merging map from robot %d into robot %d", higher, lower);

    auto &src_robot = robots[higher];
    auto &dest_robot = robots[lower];

    // Stop robots
    setRobotMotion(*dest_robot, false, *nh);
    setRobotMotion(*src_robot, false, *nh);

    // Synchronize access using std::lock to avoid deadlock
    std::unique_lock<std::mutex> lockLower(dest_robot->data_mutex, std::defer_lock);
    std::unique_lock<std::mutex> lockHigher(src_robot->data_mutex, std::defer_lock);
    std::lock(lockLower, lockHigher);

    Grid srcMap = src_robot->robotMap->getGridMap();
    Grid destMap = dest_robot->robotMap->getGridMap();

    // Merge Maps
    std::unique_ptr<map_merging>
        merge = std::make_unique<map_merging>(srcMap,
                                              dest_robot->robot_pose,
                                              destMap,
                                              src_robot->robot_pose,
                                              grid_size);
    Grid mergedMap = merge->mergeMaps();
    dest_robot->robotMap->setMap(mergedMap);
    src_robot->robotMap->setMap(mergedMap);

    // Ensure locks are held until motion is reset
    lockLower.unlock();
    lockHigher.unlock();

    // Resume motion
    setRobotMotion(*dest_robot, true, *nh);
    setRobotMotion(*src_robot, true, *nh);
}

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

    if (!robot.robotOdomObject)
    {
        robot.robotOdomObject = std::make_unique<robot_odometry>(
            robot_width,
            msg->pose.pose.position.x * 1000.0F,
            msg->pose.pose.position.y * 1000.0F,
            yaw);
    }

    robot.robot_pose = robot.robotOdomObject->m_update_pose(
        msg->pose.pose.position.x * 1000.0F,
        msg->pose.pose.position.y * 1000.0F,
        yaw);

    robot.global_pose.x = msg->pose.pose.position.x * 1000.0F;
    robot.global_pose.y = msg->pose.pose.position.y * 1000.0F;
    robot.global_pose.theta = yaw;
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
    nh = new ros::NodeHandle; // Instantiate after ros::init()

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
            robotData.robotID = i;
            robotData.robotTag = "tb3_" + std::to_string(i);
            robotData.motion_pub = nh->advertise<std_msgs::Bool>(robotData.robotTag + "/motion_control", 10);
            setRobotMotion(robotData, true, *nh);
            ros::MultiThreadedSpinner spinner(4); // Use a multi-threaded spinner
            spinner.spin(); });
    }

    std::thread interactionThread([&robots]
                                  {
        while (ros::ok()) {
            std::this_thread::sleep_for(std::chrono::seconds(MERGING_TIME_SEC)); // Slam Every 10 Seconds
            randomRobotInteraction(robots);
        } });

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
