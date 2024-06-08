#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"
#include <tf/transform_listener.h>
#include "slam/process_sensor_data.h"
#include "slam/single_robot_mapping.h"

std::unique_ptr<lidar_data> lidarScan;
ros::Publisher processed_data_pub;

void scanCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    ROS_INFO("I heard a laser scan");

    // Initialize lidarScan object using the first message
    if (!lidarScan)
    {
        lidarScan = std::make_unique<lidar_data>(msg->ranges.size(),
                                                 value_range{.min{msg->range_min}, .max{msg->range_max}},
                                                 value_range{.min{msg->angle_min}, .max{msg->angle_max}});
        ROS_INFO("Lidar Initialization Success!");
    }

    std::vector<float> scans{msg->ranges};
    sensor_msgs::LaserScan pub_msg;

    pub_msg.angle_increment = msg->angle_increment;
    pub_msg.angle_max = msg->angle_max;
    pub_msg.angle_min = msg->angle_min;
    pub_msg.range_max = msg->range_max;
    pub_msg.range_min = msg->range_min;
    pub_msg.ranges = lidarScan->process_data(scans);
    pub_msg.header.frame_id = "base_scan";
    pub_msg.intensities = msg->intensities;

    processed_data_pub.publish(pub_msg);
    std::cout << "Pub\n"
              << std::endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "slam_integration");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("scan", 1000, scanCallback);
    processed_data_pub = nh.advertise<sensor_msgs::LaserScan>("processed_lidar_data", 1000);

    ros::spin();

    return 0;
}
