/**
 * @file trajectory_generator_node.cpp
 * @brief Trajectory Generator node
 * @author Aleix Paris
 * @date 2020-01-08
 */

#include "rclcpp/rclcpp.hpp"
#include "trajectory_generator_ros2/TrajectoryGenerator.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    trajectory_generator::TrajectoryGenerator TrajectoryGenerator();
    rclcpp::spin();
    return 0;
}
