/**
 * @file trajectory_generator_node.cpp
 * @brief Trajectory Generator node
 * @author Aleix Paris
 * @date 2020-01-08
 */

#include <rclcpp/rclcpp.h>
#include "trajectory_generator/TrajectoryGenerator.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    trajectory_generator::TrajectoryGenerator TrajectoryGenerator();
    rclcpp::spin();
    return 0;
}
