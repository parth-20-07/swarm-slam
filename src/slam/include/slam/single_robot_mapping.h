/**
 * @file single_robot_mapping.h
 * @author Parth Patel (parth.pmech@gmail.com)
 * @brief
 * @version 0.1
 * @date 2024-06-06
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef SINGLE_ROBOT_MAPPING
#define SINGLE_ROBOT_MAPPING

#ifndef SINGLE_ROBOT_MAPPING_H
#define SINGLE_ROBOT_MAPPING_H

#include <algorithm>
#include <iostream>
#include <slam/process_sensor_data.h>
#include <vector>
#include <Eigen/Dense>
#include <math.h>

enum class cellState
{
    EMPTY = 0,
    UNKNOWN = 1,
    FILLED = 2,
    MERGED = 3
};

typedef Eigen::Matrix<cellState, Eigen::Dynamic, Eigen::Dynamic> Grid;

class map_environment
{
public:
    map_environment(const float &total_grid_length_millimeters, const float &grid_cell_length_millimeters);

    ~map_environment();

    Grid updateMap_robot_origin_frame(const std::vector<coordinate_t> &lidarScan, const pose_t &current_pose);

    /* --------------------------------- Getters -------------------------------- */
    [[nodiscard]] const float getGridLength() const noexcept { return this->m_totalGridLength_millimeters; }
    [[nodiscard]] const Grid &getGridMap() const noexcept { return this->m_gridMap; }
    [[nodiscard]] const size_t &getGridCellCount() const noexcept { return this->m_cellCount; }

    bool resize_grid(const float &new_grid_length_millimeters);

    int m_center;
    float m_gridCellSize_millimeters;

    void setMap(Grid &map)
    {
        m_gridMap = map;
        m_cellCount = map.rows();
        m_totalGridLength_millimeters = m_cellCount * m_gridCellSize_millimeters;
        m_center = static_cast<int>(m_cellCount / 2.0F);
    }

private:
    /* ---------------------------- Member Functions ---------------------------- */
    std::size_t calculate_grid_cell_count(const float &grid_length_millimeters);

    /* ---------------------------- Member Variables ---------------------------- */
    float m_totalGridLength_millimeters;
    size_t m_cellCount;
    Grid m_gridMap;
};
#endif // SINGLE_ROBOT_MAPPING_H

#endif /* SINGLE_ROBOT_MAPPING */
