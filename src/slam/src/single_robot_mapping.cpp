/**
 *
 * @file single_robot_mapping.cpp
 * @author Parth Patel (parth.pmech@gmail.com)
 * @brief
 * @version 0.1
 * @date 2024-06-06
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "slam/single_robot_mapping.h"

map_environment::map_environment(const float &total_grid_length_millimeters, const float &grid_cell_length_millimeters)
    : m_totalGridLength_millimeters(total_grid_length_millimeters),
      m_gridCellSize_millimeters(grid_cell_length_millimeters),
      m_cellCount(static_cast<int>(total_grid_length_millimeters / grid_cell_length_millimeters)),
      m_gridMap(m_cellCount, m_cellCount)
{
    m_center = static_cast<int>(m_cellCount / 2.0F);
    m_gridMap.setConstant(cellState::UNKNOWN);
}

map_environment::~map_environment()
{
}

bool map_environment::resize_grid(const float &new_grid_length_millimeters)
{
    if (new_grid_length_millimeters > this->m_totalGridLength_millimeters)
    {
        size_t newCellCount{calculate_grid_cell_count(new_grid_length_millimeters)};

        // Create a new grid with the new size and initialize it with UNKNOWN
        Grid newGrid(newCellCount, newCellCount);
        newGrid.setConstant(cellState::UNKNOWN);

        int offset = (newCellCount - this->m_cellCount) / 2.0F;

        // Copy the old grid into the new grid with the new offset
        for (std::size_t y = 0; y < this->m_cellCount; ++y)
        {
            for (std::size_t x = 0; x < this->m_cellCount; ++x)
            {
                newGrid(y + offset, x + offset) = this->m_gridMap(y, x);
            }
        }

        // Update the grid map with the new grid
        std::swap(this->m_gridMap, newGrid);
        // std::cout << "Resize Successful!!" << "\nCurrent size is: " << static_cast<int>(this->m_totalGridLength_millimeters) << " mm | " << static_cast<int>(m_cellCount)
        //   << "\nNew Requested Grid Size: " << static_cast<int>(new_grid_length_millimeters) << " mm| " << static_cast<int>(newCellCount) << std::endl;
        this->m_cellCount = newCellCount;
        this->m_totalGridLength_millimeters = new_grid_length_millimeters;
        this->m_center = static_cast<int>(this->m_cellCount / 2.0F);
        return true;
    }
    else
    {
        std::cout << "Cannot Resize the Grid." << "\nCurrent size is: " << static_cast<int>(this->m_totalGridLength_millimeters)
                  << "\n.New Requested Grid Size: " << static_cast<int>(new_grid_length_millimeters) << std::endl;
        return false;
    }
}

std::size_t map_environment::calculate_grid_cell_count(const float &grid_length_millimeters)
{
    return static_cast<std::size_t>(grid_length_millimeters / this->m_gridCellSize_millimeters);
}

Grid map_environment::updateMap_robot_origin_frame(const std::vector<coordinate_t> &lidarScan, const pose_t &current_pose)
{
    for (size_t i = 0; i < lidarScan.size(); i++)
    {
        auto [x_scan, y_scan] = lidarScan.at(i);
        if (x_scan != -1.0F)
        {
            // Apply rotation if needed
            float theta = current_pose.theta;
            float x = current_pose.x + (x_scan * cos(theta) - y_scan * sin(theta));
            float y = current_pose.y + (x_scan * sin(theta) + y_scan * cos(theta));

            int x_off = m_center + static_cast<int>(x / this->m_gridCellSize_millimeters);
            int y_off = m_center + static_cast<int>(y / this->m_gridCellSize_millimeters);

            // Check grid boundaries and resize if necessary
            while (x_off < 0 || x_off >= m_cellCount || y_off < 0 || y_off >= m_cellCount)
            {
                float new_length = m_totalGridLength_millimeters * 1.5F;
                resize_grid(new_length);
                x_off = m_center + static_cast<int>(x / this->m_gridCellSize_millimeters);
                y_off = m_center + static_cast<int>(y / this->m_gridCellSize_millimeters);
            }
            m_gridMap(y_off, x_off) = cellState::FILLED;
        }
    }
    return this->m_gridMap;
}
