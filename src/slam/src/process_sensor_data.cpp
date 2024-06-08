/**
 * @file process_sensor_data.cpp
 * @author Parth Patel (parth.pmech@gmail.com)
 * @brief
 * @version 0.1
 * @date 2024-06-06
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "slam/process_sensor_data.h"

lidar_data ::lidar_data(
    std::size_t number_of_lidar_data_points,
    value_range scan_distance,
    value_range angle_range)
    : m_totalLidarDataPoints(number_of_lidar_data_points),
      m_scanDistance(scan_distance),
      m_angleRange{angle_range}
{
}

lidar_data ::~lidar_data()
{
}

std::vector<float> lidar_data::process_data(std::vector<float> &data)
{
    std::vector<float> processed_data;
    processed_data.reserve(this->m_totalLidarDataPoints); // Reserve space to avoid reallocations

    for (size_t i = 0; i < this->m_totalLidarDataPoints; i++)
    {
        float val = data.at(i);
        if (val >= this->m_scanDistance.max)
        {
            val = std::numeric_limits<float>::max();
        }
        else if (val <= this->m_scanDistance.min)
        {
            val = std::numeric_limits<float>::min();
        }
        processed_data.emplace_back(val);
    }

    return std::move(processed_data);
}

robot_odometry::robot_odometry(/* args */)
{
}

robot_odometry::~robot_odometry()
{
}
