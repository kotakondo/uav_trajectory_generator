/**
 * @file Csv.hpp
 * @brief Replays a pre-generated trajectory from a CSV of snapstack_msgs2/Goal rows.
 *
 * CSV header (one row per time-slice, dt taken from the first two t values):
 *   t,x,y,z,vx,vy,vz,ax,ay,az,jx,jy,jz,yaw,dyaw,power,mode_xy,mode_z
 *
 * The vehicle's takeoff and the flight to the trajectory's first point are
 * handled by TrajectoryGenerator's flight-mode state machine (TAKING_OFF ->
 * HOVERING -> INIT_POS_TRAJ), so this class only needs to provide the goals.
 */
#pragma once

#include "trajectory_generator_ros2/trajectories/Trajectory.hpp"

#include <rclcpp/logger.hpp>
#include "snapstack_msgs2/msg/goal.hpp"

#include <string>
#include <unordered_map>
#include <vector>

namespace trajectory_generator {

class Csv : public Trajectory
{
public:
    Csv(const std::string& csv_path, double stop_accel, double dt);
    virtual ~Csv();

    void generateTraj(std::vector<snapstack_msgs2::msg::Goal>& goals,
                      std::unordered_map<int,std::string>& index_msgs,
                      const rclcpp::Clock::SharedPtr& clock) override;

    void generateStopTraj(std::vector<snapstack_msgs2::msg::Goal>& goals,
                          std::unordered_map<int,std::string>& index_msgs,
                          int& pub_index,
                          const rclcpp::Clock::SharedPtr& clock) override;

    bool trajectoryInsideBounds(double xmin, double xmax,
                                double ymin, double ymax,
                                double zmin, double zmax) override;

private:
    bool loadCsv(const std::string& path);

    std::string csv_path_;
    double stop_accel_;            // [m/s^2] deceleration used by generateStopTraj
    double csv_dt_{0.0};           // inferred sample period from the CSV t column
    bool loaded_{false};
    std::vector<snapstack_msgs2::msg::Goal> csv_goals_;

    rclcpp::Logger logger_ = rclcpp::get_logger("csv_logger");
};

} /* namespace */
