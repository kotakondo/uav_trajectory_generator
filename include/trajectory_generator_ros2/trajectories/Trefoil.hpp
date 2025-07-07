/**
 * @file Trefoil.hpp
 * @brief Trefoil class
 * @author Jialin Chen
 * @date 2025-06-18
 */

#pragma once

#include "trajectory_generator_ros2/trajectories/Trajectory.hpp"
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

#include "snapstack_msgs2/msg/goal.hpp"

#include <vector>
#include <unordered_map>
#include <Eigen/Core>
#include <string>

namespace trajectory_generator {

/*!
 * Class that represents a trefoil trajectory
 */
class Trefoil: public Trajectory
{
public:

    Trefoil(double alt, double r, double cx, double cy,
           std::vector<double> v_goals, double t_traj, double accel, double dt);

    virtual ~Trefoil();

    void generateTraj(std::vector<snapstack_msgs2::msg::Goal>& goals,
                      std::unordered_map<int,std::string>& index_msgs,
                      const rclcpp::Clock::SharedPtr& clock) override;  // generates the trefoil

    snapstack_msgs2::msg::Goal createTrefoilGoal(double v, double accel, double t) const;

    void generateStopTraj(std::vector<snapstack_msgs2::msg::Goal>& goals,
                                      std::unordered_map<int,std::string>& index_msgs,
                                      int& pub_index,
                                      const rclcpp::Clock::SharedPtr& clock) override;

    bool trajectoryInsideBounds(double xmin, double xmax,
                                double ymin, double ymax,
                                double zmin, double zmax) override;

private:
    double alt_;  // altitude in m
    double r_;  // radius in m
    double cx_, cy_;  // trefoil center in m

    std::vector<double> v_goals_;  // norm of goal velocities
    double t_traj_;  // time to follow each trajectory, s
    double accel_;  // max acceleration allowed

    rclcpp::Logger logger_ = rclcpp::get_logger("trefoil_logger");
};

} /* namespace */
