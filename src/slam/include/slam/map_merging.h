#ifndef MAP_MERGING
#define MAP_MERGING

#include <slam/single_robot_mapping.h>
#include <slam/process_sensor_data.h>
#include <memory>

class map_merging
{
public:
  map_merging(Grid &destination_map,
              const pose_t &destination_robot_pose,
              const Grid &source_map,
              const pose_t &source_robot_pose,
              const float grid_size)
      : m_destination_map(destination_map),
        m_source_map(source_map),
        m_gridSize(grid_size),
        m_transformation_from_source_to_destination_robot_frame(calculateRelativePose(source_robot_pose, destination_robot_pose))
  {
  }

  Grid mergeMaps()
  {
    // Grid transformed_src;
    // transformGrid(transformed_src, m_source_map, m_transformation_from_source_to_destination_robot_frame);

    // // Merge the transformed source into the destination map
    // for (int y = 0; y < transformed_src.rows(); ++y)
    // {
    //   for (int x = 0; x < transformed_src.cols(); ++x)
    //   {
    //     // Only merge cells that are not UNKNOWN
    //     if (transformed_src(y, x) != cellState::UNKNOWN)
    //     {
    //       while (y > m_destination_map.rows() || x > m_destination_map.cols())
    //       {
    //         resize_grid(m_destination_map);
    //       }
    //       m_destination_map(m_resizeOffset + y, m_resizeOffset + x) = cellState::MERGED;
    //     }
    //   }
    // }
    while (m_destination_map.rows() < m_source_map.rows())
    {
      resize_grid(m_destination_map);
    }

    const int dest_offset = (m_destination_map.rows() - m_source_map.rows()) / 2.0F;

    // Merge the transformed source into the destination map
    for (int y = 0; y < m_source_map.rows(); ++y)
    {
      for (int x = 0; x < m_source_map.cols(); ++x)
      {
        // Only merge cells that are not UNKNOWN
        if (m_source_map(y, x) != cellState::UNKNOWN)
        {
          const int x_off = x + dest_offset;
          const int y_off = y + dest_offset;
          m_destination_map(y_off, x_off) = m_source_map(y, x);
        }
      }
    }

    return m_destination_map;
  }

private:
  Grid m_source_map;
  Grid m_destination_map;
  Eigen::Matrix3d m_transformation_from_source_to_destination_robot_frame;
  const float m_gridSize;

private:
  Eigen::Matrix3d calculateRelativePose(pose_t dest, pose_t src)
  {
    // Calculate translation components from dest to src in the global frame
    double deltaXGlobal = src.x - dest.x;
    double deltaYGlobal = src.y - dest.y;

    // Convert dest's orientation angle from radians to ensure consistent trigonometric calculations
    double theta = -dest.theta; // Rotation needed to align global axes with dest's local axes

    // Calculate the new position of src in dest's local frame
    double deltaXLocal = deltaXGlobal * cos(theta) - deltaYGlobal * sin(theta);
    double deltaYLocal = deltaXGlobal * sin(theta) + deltaYGlobal * cos(theta);

    // Calculate relative orientation of src with respect to dest
    double deltaTheta = src.theta - dest.theta;
    deltaTheta = std::fmod(deltaTheta + M_PI, 2 * M_PI) - M_PI; // Normalize angle to range [-π, π]

    // Create the transformation matrix for src relative to dest
    Eigen::Matrix3d T;
    T << cos(deltaTheta), -sin(deltaTheta), deltaXLocal,
        sin(deltaTheta), cos(deltaTheta), deltaYLocal,
        0, 0, 1;

    return T;
  }

  void transformGrid(Grid &destination, const Grid &source, const Eigen::Matrix3d &T)
  {
    // Initialize the destination grid to match the source grid dimensions, if not already set
    if (destination.rows() != source.rows() || destination.cols() != source.cols())
    {
      destination.resize(source.rows(), source.cols());
    }

    // Set default state for all cells in the destination grid
    destination.setConstant(cellState::UNKNOWN);

    // Compute the inverse of the transformation matrix
    Eigen::Matrix3d T_inv = T.inverse();
    int rows = source.rows();
    int cols = source.cols();

    for (int i = 0; i < rows; ++i)
    {
      for (int j = 0; j < cols; ++j)
      {
        Eigen::Vector3d srcPoint(j, i, 1); // Column-major order for Eigen
        Eigen::Vector3d destPoint = T_inv * srcPoint;

        int destX = std::round(destPoint(0));
        int destY = std::round(destPoint(1));

        // Ensure the destination indices are within the bounds
        if (destX >= 0 && destX < cols && destY >= 0 && destY < rows)
        {
          // Only copy the state if it's FILLED
          if (source(i, j) == cellState::FILLED)
          {

            destination(destY, destX) = cellState::FILLED;
          }
        }
      }
    }
  }

  void resize_grid(Grid &grid)
  {
    size_t old_count = grid.rows();
    size_t newCellCount{old_count * 2.0F};

    // Create a new grid with the new size and initialize it with UNKNOWN
    Grid newGrid(newCellCount, newCellCount);
    newGrid.setConstant(cellState::UNKNOWN);

    const int resizeOffset = (newCellCount - old_count) / 2.0F;

    // Copy the old grid into the new grid with the new offset
    for (std::size_t y = 0; y < old_count; ++y)
    {
      for (std::size_t x = 0; x < old_count; ++x)
      {
        newGrid(y + resizeOffset, x + resizeOffset) = grid(y, x);
      }
    }

    // Update the grid map with the new grid
    std::swap(grid, newGrid);
  }
};

#endif /* MAP_MERGING */
