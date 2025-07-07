/**
 * @file Trefoil.cpp
 * @brief Trefoil class
 * @author Jialin Chen
 * @date 2025-07-07
 */

#include "trajectory_generator_ros2/trajectories/Trefoil.hpp"

#include "snapstack_msgs2/msg/quad_flight_mode.hpp"
#include "snapstack_msgs2/msg/state.hpp"
#include "snapstack_msgs2/msg/goal.hpp"

#include <vector>
#include <unordered_map>
#include <string>
#include <Eigen/Core>
#include <cmath>

namespace trajectory_generator {

Trefoil::Trefoil(double alt, double r, double cx, double cy,
           std::vector<double> v_goals, double t_traj, double accel, double dt):
      alt_(alt), r_(r), cx_(cx), cy_(cy),
      v_goals_(v_goals), t_traj_(t_traj), accel_(accel), Trajectory(dt){}
      
Trefoil::~Trefoil()
{
}

void Trefoil::generateTraj(std::vector<snapstack_msgs2::msg::Goal>& goals,
                          std::unordered_map<int,std::string>& index_msgs,
                          const rclcpp::Clock::SharedPtr& clock)
{
    // rclcpp::Time tstart = rclcpp::Time::now();
    rclcpp::Time tstart = clock->now();

    // init pos
    double t = 0.0;
    double v = 0.0;

    goals.push_back(createTrefoilGoal(v, 0, t));

    for(int i = 0; i < v_goals_.size(); ++i){
        double v_goal = v_goals_[i];
        index_msgs[goals.size() - 1] = "Trefoil traj: accelerating to " + std::to_string(v_goal) + " m/s";
        // accelerate to the goal velocity
        while(v < v_goal){
            // generate points in the Trefoil with *increasing* velocity
            v = std::min(v + accel_*dt_, v_goal);
            goals.push_back(createTrefoilGoal(v, accel_, t));
            t += dt_;

        }

        // we reached v_goal, continue the traj for t_traj_ seconds
        if(fabs(v - v_goal) > 0.001){
            RCLCPP_WARN(logger_, "Vels are not in increasing order, ignoring vels from the first to decrease...");
        }

        index_msgs[goals.size() - 1] = "Trefoil traj: reached " + std::to_string(v_goal)
                + " m/s, keeping constant v for " + std::to_string(t_traj_) + " s";
        double current_t_traj_ = 0;
        while(current_t_traj_ < t_traj_){
            // generate points in the Trefoil with *constant* velocity v == v_goal
            goals.push_back(createTrefoilGoal(v, 0, t));
            t += dt_;
            current_t_traj_ += dt_;
        }
    }
    // now decelerate to 0
    index_msgs[goals.size() - 1] = "Trefoil traj: decelerating to 0 m/s";
    while(v > 0){
        // generate points in the Trefoil with *decreasing* velocity
        v = std::max(v - accel_*dt_, 0.0);
        goals.push_back(createTrefoilGoal(v, -accel_, t));
        t += dt_;

    }

    // we reached zero velocity
    if(fabs(v) > 0.001){
        RCLCPP_ERROR(logger_, "Error: final velocity is not zero");
        exit(1);
    }
    index_msgs[goals.size() - 1] = "Trefoil traj: stopped";

    // RCLCPP_INFO(logger_, "Time to calculate the traj (s): %f", (rclcpp::Time::now() - tstart).seconds());
    RCLCPP_INFO(logger_, "Time to calculate the traj (s): %f", (clock->now() - tstart).seconds());
    RCLCPP_INFO(logger_, "Goal vector size = %lu", goals.size());
}

snapstack_msgs2::msg::Goal Trefoil::createTrefoilGoal(double v, double accel, double t) const{
    // TODO: return ref to goal to avoid copy? Same for the simpleInterpolation function
 
    // double v2r  = pow(v,2)/r_;
    // double v3r2 = pow(v,3)/pow(r_,2);
    // double v4r3 = pow(v,4)/pow(r_,3);
    

    snapstack_msgs2::msg::Goal goal;
    goal.header.frame_id = "world";
    goal.p.x   = cx_ + r_ * (sin(t) + 2 * sin(2*t));
    goal.p.y   = cy_ + r_ * (cos(t) - 2 * cos(2*t));
    goal.p.z   = alt_ + r_ * (-sin(3*t));
    goal.v.x   = r_ * (cos(t) + 4 * cos(2*t));
    goal.v.y   = r_ * (-sin(t) + 4 * sin(2*t));
    goal.v.z   = r_ * (-3 * cos(3*t));
    goal.a.x = r_ * (-sin(t) - 8 * sin(2*t)); //+ accel*c; //this accel term makes a.x discontinuous
    goal.a.y = r_ * (-cos(t) + 8 * cos(2*t)); //+ accel*s; //this accel term makes a.y discontinuous
    goal.a.z = r_ * (9 * sin(3*t));
    goal.j.x  = 0;
    goal.j.y  = 0;
    goal.j.z  = 0;
    //goal.s.x  = v4r3*c;
    //goal.s.y  = v4r3*s;
    //goal.s.z  = 0;
    goal.psi = atan2(goal.v.y, goal.v.x); // face direction of velocity
    goal.dpsi = v / r_;  // dyaw has to be in world frame, the outer loop already converts it to body
    goal.power = true;

    return goal;
}

void Trefoil::generateStopTraj(std::vector<snapstack_msgs2::msg::Goal>& goals,
                              std::unordered_map<int,std::string>& index_msgs,
                              int& pub_index,
                              const rclcpp::Clock::SharedPtr& clock)
{
    rclcpp::Time tstart = clock->now();

    const auto& current_goal = goals[pub_index];
    Eigen::Vector3d current_pos(current_goal.p.x, current_goal.p.y, current_goal.p.z);

    // Brute-force search to estimate closest `t` on the trefoil
    double best_t = 0.0;
    double min_dist = 1e9;
    for (double test_t = 0; test_t <= 6*M_PI; test_t += 0.01) {
        double x = cx_ + r_ * (sin(test_t) + 2 * sin(2*test_t));
        double y = cy_ + r_ * (cos(test_t) - 2 * cos(2*test_t));
        double z = alt_ + r_ * (-sin(3*test_t));
        Eigen::Vector3d test_pos(x, y, z);
        double dist = (test_pos - current_pos).norm();
        if (dist < min_dist) {
            min_dist = dist;
            best_t = test_t;
        }
    }

    double v = std::sqrt(std::pow(current_goal.v.x, 2) +
                         std::pow(current_goal.v.y, 2) +
                         std::pow(current_goal.v.z, 2));

    double t = best_t;
    std::vector<snapstack_msgs2::msg::Goal> goals_tmp;
    std::unordered_map<int,std::string> index_msgs_tmp;

    index_msgs_tmp[0] = "Trefoil traj: pressed END, decelerating to 0 m/s";

    while (v > 0) {
        v = std::max(v - accel_ * dt_, 0.0);
        goals_tmp.push_back(createTrefoilGoal(v, -accel_, t));
        t += dt_;
    }

    index_msgs_tmp[goals_tmp.size() - 1] = "Trefoil traj: stopped";

    goals = std::move(goals_tmp);
    index_msgs = std::move(index_msgs_tmp);
    pub_index = 0;

    RCLCPP_INFO(logger_, "Time to calculate the braking traj (s): %f", (clock->now() - tstart).seconds());
    RCLCPP_INFO(logger_, "Goal vector size = %lu", goals.size());
}


bool Trefoil::trajectoryInsideBounds(double xmin, double xmax,
                                     double ymin, double ymax,
                                     double zmin, double zmax)
{
    const double t_start = 0.0;
    const double t_end = 6 * M_PI;  // Full trefoil
    const double t_step = 0.1;

    for (double t = t_start; t <= t_end; t += t_step) {
        double x = cx_ + r_ * (sin(t) + 2 * sin(2*t));
        double y = cy_ + r_ * (cos(t) - 2 * cos(2*t));
        double z = alt_ + r_ * (-sin(3*t));

        if (!isPointInsideBounds(xmin, xmax, ymin, ymax, zmin, zmax,
                                 Eigen::Vector3d(x, y, z))) {
            return false;  // At least one point out of bounds
        }
    }
    return true;
}


} /* namespace */
