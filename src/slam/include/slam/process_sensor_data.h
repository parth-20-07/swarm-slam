/**
 * @file process_sensor_data.h
 * @author Parth Patel (parth.pmech@gmail.com)
 * @brief
 * @version 0.1
 * @date 2024-06-06
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef PROCESS_SENSOR_DATA_H
#define PROCESS_SENSOR_DATA_H
#include <iostream>
#include <stdint.h>
#include <vector>
#include <limits>

struct value_range
{
    float min;
    float max;
};

/**
 * @brief
 * TODO: Implement Interface to take in ArGos Data
 * TODO: Implement LIDAR Data cleaning Algorithm
 * TODO: Implement Interface to feed data for SLAM
 *
 */
class lidar_data
{
private:
    /* ---------------------------- Member Functions ---------------------------- */

    /* ---------------------------- Member Variables ---------------------------- */
    std::size_t m_totalLidarDataPoints; // Angle Increment is assumed Uniform
    struct value_range m_scanDistance   // Lidar Range
    {
        .min{0.0F}, .max { 0.0F }
    };
    struct value_range m_angleRange // Start and End Angle for LIDAR
    {
        .min{0.0F}, .max { 360.0F }
    };

public:
    /* ---------------------------- Member Functions ---------------------------- */
    lidar_data(
        std::size_t number_of_lidar_data_points,
        value_range scan_distance,
        value_range angle_range);
    ~lidar_data();

    std::vector<float> process_data(std::vector<float> &data);
    /* ---------------------------- Member Variables ---------------------------- */
};

/**
 * @brief
 * TODO: Write Interface for KheperaIV
 * TODO: Implement Functionality for Robot Odometry
 * TODO: Implement Location/Pose Estimation Functionality
 * TODO: Implement Interface to feed data to SLAM
 */
class robot_odometry
{
private:
    /* ---------------------------- Member Functions ---------------------------- */

    /* ---------------------------- Member Variables ---------------------------- */
public:
    /* ---------------------------- Member Functions ---------------------------- */
    robot_odometry(/* args */);
    ~robot_odometry();

    /* ---------------------------- Member Variables ---------------------------- */
};

#endif // PROCESS_SENSOR_DATA_H
