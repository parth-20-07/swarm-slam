/**
 * @file process_sensor_data.cpp
 * @brief Implementation file for processing sensor data.
 * @version 0.1
 * @date 2024-06-06
 */

#include "slam/process_sensor_data.h"
#include <mutex>
#include <thread>
#include <tuple>
#include <math.h>

// Constructor for lidar_data
lidar_data::lidar_data(
    std::size_t number_of_lidar_data_points,
    value_range_t scan_distance,
    float detection_threshold,
    const pose_t transformation_to_robot_frame,
    const float mountingAngleOffset,
    const float multiplier)
    : m_totalLidarDataPoints(number_of_lidar_data_points),
      m_scanDistance(scan_distance),
      m_detectionThreshold(detection_threshold),
      m_anglePerRayIncrement_Degrees(number_of_lidar_data_points / 360.0F),
      m_transformation_to_robot_frame(transformation_to_robot_frame),
      m_mountingAngleOffset(mountingAngleOffset),
      m_multiplier(multiplier)
{
}

// Destructor for lidar_data
lidar_data::~lidar_data() = default;

std::vector<float> lidar_data::calculate_derivative(const std::vector<float> &data) const
{
    std::vector<float> derivative;
    derivative.resize(data.size());
    derivative.at(0) = 0.0F;
    for (std::size_t idx = 1; idx < data.size() - 2; idx++)
    {
        float left = data.at(idx - 1);
        float right = data.at(idx + 1);
        if (right > m_scanDistance.min && left > m_scanDistance.min)
        {
            derivative.at(idx) = (right - left) / 2.0F;
        }
        else
        {
            derivative.at(idx) = 0.0F;
        }
    }
    derivative.at(data.size() - 1) = 0.0F;
    return derivative;
}

std::vector<obstacle_location_t> lidar_data::find_obstacles(
    const std::vector<float> &data,
    const std::vector<float> &derivative_data) const
{
    std::vector<obstacle_location_t> all_obstacles; // All the Obstacles in the scan

    for (std::size_t idx = 0; idx < data.size(); idx++)
    {
        if (derivative_data.at(idx) < 0.0F)
        {
            // Low Trigger Detected for Obstacle
            std::vector<obstacle_location_t> obstacles_in_derivative; // Obstacle Indexes in the current derivative
            while (idx < data.size() && derivative_data.at(idx) <= 0)
            {
                if (data.at(idx) > m_scanDistance.min)
                {
                    obstacle_location_t obstacle{.ray_index = idx, .depth = data.at(idx)};
                    obstacles_in_derivative.push_back(obstacle);
                }
                idx++;
            }

            if (idx == data.size())
                break;
            // if (derivative_data.at(idx) > 0) {
            // Obstacle Reading Complete
            for (obstacle_location_t obstacle : obstacles_in_derivative)
                all_obstacles.push_back(obstacle);
            // } else {
            //     //Detected Another Obstacle in front of current one
            // }
        }
    }
    return all_obstacles;
}

[[nodiscard]] std::vector<coordinate_t> lidar_data::convert_obstacle_to_coordinate_in_lidar_frame(
    const std::vector<obstacle_location_t> &obstacles,
    const pose_t currentPose) const
{
    std::vector<coordinate_t> obstacle_positions;
    for (std::size_t idx = 0; idx < obstacles.size(); idx++)
    {
        auto obstacle = obstacles.at(idx);
        float ray_id = obstacle.ray_index;
        float ray_value = obstacle.depth;
        if (ray_value < m_scanDistance.max)
            obstacle_positions.emplace_back(this->m_convert_ray_to_position(
                currentPose,
                ray_id,
                ray_value));
    }

    return obstacle_positions;
}

[[nodiscard]] std::vector<coordinate_t> lidar_data::convert_scan_to_coordinate_in_lidar_frame(
    const std::vector<float> &data,
    const pose_t currentPose) const
{
    std::vector<coordinate_t> positions;
    for (std::size_t idx = 0; idx < data.size(); idx++)
    {
        float ray_id = idx;
        float ray_value = data.at(idx);
        if (ray_value < m_scanDistance.max)
            positions.emplace_back(this->m_convert_ray_to_position(currentPose, ray_id, ray_value));
    }

    return positions;
}

[[nodiscard]] std::vector<coordinate_t> lidar_data::process_lidar_scan_to_robot_frame(
    std::vector<float> &lidar_scan,
    const pose_t current_pose)
{
    for (size_t idx = 0; idx < lidar_scan.size(); idx++)
    {
        if (isnan(lidar_scan.at(idx)) || lidar_scan.at(idx) == INFINITY)
        {
            lidar_scan.at(idx) = this->m_scanDistance.max;
        }
    }

    auto lidar_scan_in_cartesian_frame = this->convert_scan_to_coordinate_in_lidar_frame(lidar_scan, current_pose);
    auto lidar_scan_robot_frame = this->transform_from_lidar_frame_to_robot_frame(lidar_scan_in_cartesian_frame);
    return (std::move(lidar_scan_robot_frame));
}

[[nodiscard]] std::vector<coordinate_t> lidar_data::get_coordinates_in_robot_origin_frame(
    const std::vector<coordinate_t> local_frame_coordinates,
    const pose_t current_pose)
{
    std::vector<coordinate_t> robot_frame_coordinates;
    for (auto [x_coord, y_coord] : local_frame_coordinates)
    {
        if (x_coord != -1.0F)
        {
            // Apply rotation if needed
            float theta = current_pose.theta;
            float x = current_pose.x + (x_coord * std::cos(theta) - y_coord * sin(theta));
            float y = current_pose.y + (x_coord * sin(theta) + y_coord * cos(theta));
            robot_frame_coordinates.emplace_back(coordinate_t{.x = x, .y = y});
        }
    }
    return robot_frame_coordinates;
}

[[nodiscard]] std::vector<coordinate_t> lidar_data::transform_from_lidar_frame_to_robot_frame(
    const std::vector<coordinate_t> local_frame_coordinates)
{
    std::vector<coordinate_t> robot_frame_coordinates;
    for (auto [x_coord, y_coord] : local_frame_coordinates)
    {
        if (x_coord != -1.0F)
        {
            // Apply rotation if needed
            float theta = m_transformation_to_robot_frame.theta;
            float x = m_transformation_to_robot_frame.x + (x_coord * std::cos(theta) - y_coord * sin(theta));
            float y = m_transformation_to_robot_frame.y + (x_coord * sin(theta) + y_coord * cos(theta));
            robot_frame_coordinates.emplace_back(coordinate_t{.x = x, .y = y});
        }
    }
    return robot_frame_coordinates;
}

coordinate_t lidar_data::m_convert_ray_to_position(pose_t current_pose, int ray_id, float ray_value) const
{
    static const int x_axis_ray_id = 0;
    const float alpha = (ray_id)*m_anglePerRayIncrement_Degrees * M_PI / 180.0F; // Angle of the current ray
    static const float half_FoV = (static_cast<float>(m_totalLidarDataPoints) / 2.0F) * m_anglePerRayIncrement_Degrees;
    const float gamma = half_FoV - alpha; // Adjusting alpha to the lidar frame
    // const float gamma = (ray_id - 330.0) * 0.006135923151543 + m_mountingAngleOffset;

    const float dX = ray_value * m_multiplier * std::cos(alpha);
    const float dY = ray_value * m_multiplier * std::sin(alpha);

    return coordinate_t{dX, dY};
}

// Constructor for robot_odometry
robot_odometry::robot_odometry(
    const float encoder_ticks_per_mm,
    const float distance_between_wheels_in_mm,
    const pose_t starting_pose,
    const encoder_ticks_t starting_encoder_ticks)
    : m_encoderTicksPerMillimeter(encoder_ticks_per_mm),
      m_robotWidthBetweenWheels_millimeters(distance_between_wheels_in_mm),
      m_currentEncoderTickValue(starting_encoder_ticks),
      m_robotPose(starting_pose)
{
}

robot_odometry::robot_odometry(float distance_between_wheels_in_mm, float xStart, float yStart, float thetaStart)
    : m_encoderTicksPerMillimeter(0),
      m_robotWidthBetweenWheels_millimeters(distance_between_wheels_in_mm),
      m_startPose(xStart, yStart, thetaStart)
{
}

// Destructor for robot_odometry
robot_odometry::~robot_odometry() = default;

// Update pose using x, y, and theta
pose_t robot_odometry::m_update_pose(const float x, const float y, const float theta)
{
    double dx = x - m_startPose.x;
    double dy = y - m_startPose.y;

    // Rotation transformation with respect to the initial orientation
    double cos_theta = cos(-m_startPose.theta);
    double sin_theta = sin(-m_startPose.theta);

    this->m_robotPose.x = cos_theta * dx - sin_theta * dy;
    this->m_robotPose.y = sin_theta * dx + cos_theta * dy;

    // Normalize the angle
    double delta_theta = theta - m_startPose.theta;
    this->m_robotPose.theta = fmod(delta_theta, 2 * M_PI);
    if (this->m_robotPose.theta > M_PI)
        this->m_robotPose.theta -= 2 * M_PI;
    else if (this->m_robotPose.theta < -M_PI)
        this->m_robotPose.theta += 2 * M_PI;

    return this->m_robotPose;
}

// Update pose using encoder ticks
pose_t robot_odometry::m_update_pose(const encoder_ticks_t new_encoder_ticks)
{
    const float dL = (new_encoder_ticks.left - this->m_currentEncoderTickValue.left) * this->m_encoderTicksPerMillimeter;
    const float dR = (new_encoder_ticks.right - this->m_currentEncoderTickValue.right) * this->m_encoderTicksPerMillimeter;
    this->m_calculate_motion(dL, dR);
    this->m_currentEncoderTickValue = new_encoder_ticks;
    return this->m_robotPose;
}

// Calculate motion based on wheel distances
void robot_odometry::m_calculate_motion(const float dL, const float dR)
{
    float newTheta, newX, newY;

    if (dL == dR)
    {
        // Motion is translational
        newX = this->m_robotPose.x + (dL * std::cos(this->m_robotPose.theta));
        newY = this->m_robotPose.y + (dL * std::sin(this->m_robotPose.theta));
        newTheta = this->m_robotPose.theta;
    }
    else
    {
        // Motion is rotational
        const float alpha = (dR - dL) / this->m_robotWidthBetweenWheels_millimeters;
        const float radiusOfRotation = dL / alpha;
        const float RDistance = radiusOfRotation + (this->m_robotWidthBetweenWheels_millimeters / 2.0F);

        const float cX = this->m_robotPose.x - (RDistance * std::sin(this->m_robotPose.theta));
        const float cY = this->m_robotPose.y + (RDistance * std::cos(this->m_robotPose.theta));

        newTheta = std::fmod((this->m_robotPose.theta + alpha), (2.0F * M_PI));
        newX = cX + (RDistance * std::sin(newTheta));
        newY = cY - (RDistance * std::cos(newTheta));
    }
    this->m_robotPose = pose_t{newX, newY, newTheta};
}

//////////////////////////////////// Localization ////////////////////////////////////////////

std::vector<float> calculateDistances(const std::vector<coordinate_t> &obstacles, float grid_size);

// Calculate distances between obstacles
std::vector<float> calculateDistances(const std::vector<coordinate_t> &obstacles, float grid_size)
{
    std::vector<float> distances;

    for (size_t i = 0; i < obstacles.size(); ++i)
    {
        for (size_t j = i + 1; j < obstacles.size(); ++j)
        {
            float dx = (obstacles[j].x - obstacles[i].x) / grid_size;
            float dy = (obstacles[j].y - obstacles[i].y) / grid_size;
            float distance = std::sqrt(dx * dx + dy * dy);
            distances.push_back(distance);
        }
    }

    return distances;
}
