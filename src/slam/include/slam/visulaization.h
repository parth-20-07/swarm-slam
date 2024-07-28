#ifndef VISULAIZATION
#define VISULAIZATION

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include "slam/process_sensor_data.h"
#include "slam/single_robot_mapping.h"

namespace Viz
{
    static const cv::Scalar red(0, 0, 255);
    static const cv::Scalar green(0, 255, 0);
    static const cv::Scalar blue(255, 0, 0);
    static const cv::Scalar white(255, 255, 255);

    void plotGraphPoints(
        cv::Mat &image,
        const std::vector<pose_t> &pose,
        const cv::Scalar &color,
        int height,
        const std::unique_ptr<map_environment> &robotMap)
    {
        for (const auto &[x, y, theta] : pose)
        {
            const int x_off = robotMap->m_center + static_cast<int>(x / robotMap->m_gridCellSize_millimeters);
            const int y_off = height - (robotMap->m_center + static_cast<int>(y / robotMap->m_gridCellSize_millimeters)) - 1;
            cv::circle(image, cv::Point(x_off, y_off), 2, color, cv::FILLED);

            // Calculate the end point of the heading line
            const int line_length = 10; // length of the line indicating heading
            const int x_end = x_off + static_cast<int>(line_length * cos(theta));
            const int y_end = y_off - static_cast<int>(line_length * sin(theta));

            // Draw the heading line
            cv::line(image, cv::Point(x_off, y_off), cv::Point(x_end, y_end), color, 1);
        }
    }

    void plotLinesObstacles(
        cv::Mat &image,
        const std::vector<coordinate_t> &coordinates,
        const cv::Scalar &color,
        int height,
        const std::unique_ptr<map_environment> &robotMap)
    {
        // Draw lines
        for (size_t i = 1; i < coordinates.size(); ++i)
        {
            auto [x1, y1] = coordinates.at(i - 1);
            auto [x2, y2] = coordinates.at(i);
            const int xCoord1 = robotMap->m_center + static_cast<int>(x1 / robotMap->m_gridCellSize_millimeters);
            const int yCoord1 = height - (robotMap->m_center + static_cast<int>(y1 / robotMap->m_gridCellSize_millimeters)) - 1;
            const int xCoord2 = robotMap->m_center + static_cast<int>(x2 / robotMap->m_gridCellSize_millimeters);
            const int yCoord2 = height - (robotMap->m_center + static_cast<int>(y2 / robotMap->m_gridCellSize_millimeters)) - 1;
            cv::line(image, cv::Point(xCoord1, yCoord1), cv::Point(xCoord2, yCoord2), color, 2);
        }
    }

    void gridToImage(
        cv::Mat &image,
        const Grid &grid)
    {
        const int rows = grid.rows();
        const int cols = grid.cols();
        image = cv::Mat(rows, cols, CV_8UC3);

        // Define colors for each cell state
        static const cv::Vec3b emptyColor(255, 255, 255);   // White for EMPTY
        static const cv::Vec3b unknownColor(127, 127, 127); // Grey for UNKNOWN
        static const cv::Vec3b filledColor(0, 0, 0);        // Black for FILLED
        static const cv::Vec3b mergedColor(255, 0, 0);      // Blue for FILLED

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
                    case cellState::MERGED:
                        image.at<cv::Vec3b>(y_val, x) = mergedColor;
                        break;
                    default:
                        std::cout << "Unknown cell state\n";
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

    void drawLegend(
        cv::Mat &image,
        const std::vector<std::string> &labels,
        const std::vector<cv::Scalar> &colors)
    {
        static constexpr int fontFace = cv::FONT_HERSHEY_SIMPLEX;
        static constexpr int x = 50;
        static constexpr int y = 50;
        for (size_t i = 0; i < labels.size(); ++i)
        {
            constexpr int thickness = 1;
            constexpr double fontScale = 0.5;
            cv::putText(image, labels[i], cv::Point(x + 20, y + i * 20), fontFace, fontScale, white, thickness);
            cv::rectangle(image, cv::Point(x, y + i * 20 - 10), cv::Point(x + 10, y + i * 20), colors[i], cv::FILLED);
        }
    }
};

#endif /* VISULAIZATION */
