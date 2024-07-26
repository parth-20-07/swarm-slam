#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include <tf/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <opencv2/opencv.hpp>
#include <fmt/core.h>
#include "slam/process_sensor_data.h"
#include "slam/single_robot_mapping.h"
#include <boost/bind.hpp>
///////////////////// Global Variables /////////////////////
cv::Scalar red(0, 0, 255);
cv::Scalar green(0, 255, 0);
cv::Scalar blue(255, 0, 0);
cv::Scalar white(255, 255, 255);
///////////////////// Robot Specific Variables /////////////////////
/// Robot Dimensions
constexpr float robot_width = 160; // mm
/// LIDAR Specific Variables
constexpr std::size_t lidar_data_points_counts = 360;
const pose_t lidar_to_robot_transformation{0.0F, 0.0F, 0.0F};
/////////////////////////// Mapping Variables /////////////////////////////
constexpr float grid_size = 10.0f;

template <typename T, typename... Vectors>
int computeImageDimensions(const std::vector<T> &first, const Vectors &...others)
{
    auto minmax_x = std::minmax_element(first.begin(), first.end());
    T min_x = *minmax_x.first;
    T max_x = *minmax_x.second;
    ([&](const std::vector<T> &vec)
     {
        auto minmax = std::minmax_element(vec.begin(), vec.end());
        min_x = std::min(min_x, *minmax.first);
        max_x = std::max(max_x, *minmax.second); }(others), ...);
    return static_cast<int>(max_x - min_x + 100);
}

template <typename T>
void plotGraphPoints(cv::Mat &image, const std::vector<T> &x, const std::vector<T> &y, const cv::Scalar &color,
                     int height)
{
    // Draw points
    for (size_t i = 0; i < x.size(); ++i)
    {
        int xCoord = static_cast<int>(x[i]);
        int yCoord = height - static_cast<int>(y[i]);
        cv::circle(image, cv::Point(xCoord, yCoord), 2, color, cv::FILLED);
    }
}

void plotGraphPoints(cv::Mat &image, const std::vector<pose_t> &pose, const cv::Scalar &color, int height,
                     const std::unique_ptr<map_environment> &robotMap)
{
    for (const auto &[x, y, theta] : pose)
    {
        const int x_off = robotMap->m_center + static_cast<int>(x / grid_size);
        const int y_off = height - (robotMap->m_center + static_cast<int>(y / grid_size)) - 1;
        cv::circle(image, cv::Point(x_off, y_off), 2, color, cv::FILLED);

        // Calculate the end point of the heading line
        constexpr int line_length = 10; // length of the line indicating heading
        const int x_end = x_off + static_cast<int>(line_length * cos(theta));
        const int y_end = y_off - static_cast<int>(line_length * sin(theta));

        // Draw the heading line
        cv::line(image, cv::Point(x_off, y_off), cv::Point(x_end, y_end), color, 1);
    }
}

void plotGraphPoints(cv::Mat &image, const std::vector<coordinate_t> &coordinates, const cv::Scalar &color,
                     int height, const std::unique_ptr<map_environment> &robotMap)
{
    for (const auto &[x, y] : coordinates)
    {
        const int x_off = robotMap->m_center + static_cast<int>(x / grid_size);
        const int y_off = height - (robotMap->m_center + static_cast<int>(y / grid_size)) - 1;
        cv::circle(image, cv::Point(x_off, y_off), 10, color, cv::FILLED);
    }
}

template <typename T>
void plotLineGraph(cv::Mat &image, const std::vector<T> &x, const std::vector<T> &y, const cv::Scalar &color,
                   int height)
{
    // Draw lines
    for (size_t i = 1; i < x.size(); ++i)
    {
        int xCoord1 = static_cast<int>(x[i - 1]);
        int yCoord1 = height - static_cast<int>(y[i - 1]);
        int xCoord2 = static_cast<int>(x[i]);
        int yCoord2 = height - static_cast<int>(y[i]);
        cv::line(image, cv::Point(xCoord1, yCoord1), cv::Point(xCoord2, yCoord2), color, 2);
    }
}

void plotLineGraph(cv::Mat &image, const std::vector<coordinate_t> &coordinates, const cv::Scalar &color, int height)
{
    // Draw lines
    for (size_t i = 1; i < coordinates.size(); ++i)
    {
        auto [x1, y1] = coordinates.at(i - 1);
        auto [x2, y2] = coordinates.at(i);
        const int xCoord1 = height - static_cast<int>(x1);
        const int yCoord1 = height - static_cast<int>(y1);
        const int xCoord2 = height - static_cast<int>(x2);
        const int yCoord2 = height - static_cast<int>(y2);
        cv::line(image, cv::Point(xCoord1, yCoord1), cv::Point(xCoord2, yCoord2), color, 5);
    }
}

void plotLinesObstacles(cv::Mat &image, const std::vector<coordinate_t> &coordinates, const cv::Scalar &color,
                        int height, const std::unique_ptr<map_environment> &robotMap)
{
    // Draw lines
    for (size_t i = 1; i < coordinates.size(); ++i)
    {
        auto [x1, y1] = coordinates.at(i - 1);
        auto [x2, y2] = coordinates.at(i);
        const int xCoord1 = robotMap->m_center + static_cast<int>(x1 / grid_size);
        const int yCoord1 = height - (robotMap->m_center + static_cast<int>(y1 / grid_size)) - 1;
        const int xCoord2 = robotMap->m_center + static_cast<int>(x2 / grid_size);
        const int yCoord2 = height - (robotMap->m_center + static_cast<int>(y2 / grid_size)) - 1;
        cv::line(image, cv::Point(xCoord1, yCoord1), cv::Point(xCoord2, yCoord2), color, 2);
    }
}

void gridToImage(cv::Mat &image, const Grid &grid)
{
    const int rows = grid.rows();
    const int cols = grid.cols();
    image = cv::Mat(rows, cols, CV_8UC3);

    // Define colors for each cell state
    static const cv::Vec3b emptyColor(255, 255, 255);   // White for EMPTY
    static const cv::Vec3b unknownColor(127, 127, 127); // Grey for UNKNOWN
    static const cv::Vec3b filledColor(0, 0, 0);        // Black for FILLED

    // Populate the image
    for (int y = 0; y < rows; y++)
    {
        const int y_val = rows - y - 1;
        for (int x = 0; x < cols; x++)
        {
            // Ensure y and x are within bounds
            if (y < rows && x < cols)
            {
                switch (grid(y, x))
                {
                case cellState::EMPTY:
                    image.at<cv::Vec3b>(y_val, x) = emptyColor;
                    break;
                case cellState::UNKNOWN:
                    image.at<cv::Vec3b>(y_val, x) = unknownColor;
                    break;
                case cellState::FILLED:
                    image.at<cv::Vec3b>(y_val, x) = filledColor;
                    break;
                default:
                    ROS_WARN("Unknown cell state");
                    break;
                }
            }
            else
            {
                std::cerr << "Index out of bounds: (" << y << ", " << x << ")" << std::endl;
            }
        }
    }
}

void drawLegend(cv::Mat &image, const std::vector<std::string> &labels, const std::vector<cv::Scalar> &colors,
                int x = 50, int y = 50)
{
    static constexpr int fontFace = cv::FONT_HERSHEY_SIMPLEX;

    for (size_t i = 0; i < labels.size(); ++i)
    {
        constexpr int thickness = 1;
        constexpr double fontScale = 0.5;
        cv::putText(image, labels[i], cv::Point(x + 20, y + i * 20), fontFace, fontScale, white, thickness);
        cv::rectangle(image, cv::Point(x, y + i * 20 - 10), cv::Point(x + 10, y + i * 20), colors[i], cv::FILLED);
    }
}

void lidarCallback(const sensor_msgs::LaserScan::ConstPtr &msg,
                   std::unique_ptr<lidar_data> &lidarObject,
                   std::unique_ptr<map_environment> &robotMap,
                   pose_t &current_pose,
                   std::vector<pose_t> &all_pose_robot_global_frame,
                   cv::Mat &slam_map_image)
{
    ROS_INFO("Lidar Callback");
    // Initialize lidarScan object using the first message
    if (!lidarObject)
    {
        lidarObject = std::make_unique<lidar_data>(
            msg->ranges.size(),
            value_range_t{.min = msg->range_min, .max = msg->range_max},
            value_range_t{.min = -M_PI, .max = M_PI},
            0.650F,
            lidar_to_robot_transformation,
            0.0F,
            1000.0F);
        robotMap = std::make_unique<map_environment>(1000.0, grid_size);
        ROS_INFO("Lidar Initialization Success!");
    }

    std::vector<float> scans{msg->ranges};
    const auto [lidar_map_robot_frame, obstacles_robot_frame, obstacle_lidar_frame] = lidarObject->process_lidar_scan_to_robot_frame(
        scans, current_pose);
    auto currentMap_robot_origin_frame = robotMap->updateMap_robot_origin_frame(
        obstacles_robot_frame, current_pose);

    all_pose_robot_global_frame.push_back(current_pose);

    const auto global_frame_obstacles = lidarObject->get_coordinates_in_robot_origin_frame(
        obstacles_robot_frame, current_pose);
    const auto global_frame_lidar = lidarObject->get_coordinates_in_robot_origin_frame(
        lidar_map_robot_frame, current_pose);

    // std::cout << "Frame: " << std::to_string(idx) << "\n";
    gridToImage(slam_map_image, currentMap_robot_origin_frame);
    plotGraphPoints(slam_map_image, all_pose_robot_global_frame, red, slam_map_image.cols, robotMap);
    plotLinesObstacles(slam_map_image, global_frame_obstacles, green, slam_map_image.cols, robotMap);
    plotGraphPoints(slam_map_image, global_frame_obstacles, blue, slam_map_image.cols, robotMap);
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
        return;
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
                                                                         boost::ref(lidarObject),
                                                                         boost::ref(robotMap),
                                                                         boost::ref(robot_pose),
                                                                         boost::ref(all_pose_robot_global_frame),
                                                                         boost::ref(slam_map_image)));
    ros::spin();
    cv::destroyWindow("Lidar Map");

    return 0;
}
