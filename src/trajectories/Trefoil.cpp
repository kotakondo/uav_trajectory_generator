/**
 * @file Trefoil.cpp
 * @brief Trefoil class
 * @author Jialin Chen
 * @date 2025-06-18
 */

#include "trajectory_generator_ros2/trajectories/Trefoil.hpp"

#include "snapstack_msgs2/msg/quad_flight_mode.hpp"
#include "snapstack_msgs2/msg/state.hpp"
#include "snapstack_msgs2/msg/goal.hpp"

#include <vector>
#include <unordered_map>
#include <string>
#include <Eigen/Core>

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
    rclcpp::Time tstart = clock->now();

    // init pos
    double theta = 0;
    double v = 0;

    goals.push_back(createTrefoilGoal(v, 0, theta));

    for(int i = 0; i < v_goals_.size(); ++i){
        double v_goal = v_goals_[i];
        index_msgs[goals.size() - 1] = "Trefoil traj: accelerating to " + std::to_string(v_goal) + " m/s";
        
        // accelerate to the goal velocity
        while(v < v_goal){
            // generate points in the Trefoil with *increasing* velocity
            v = std::min(v + accel_*dt_, v_goal);
            
            // Use consistent theta advancement
            double vx_tangent = r_ * (cos(theta) + 4 * cos(2*theta));
            double vy_tangent = r_ * (-sin(theta) + 4 * sin(2*theta));
            double vz_tangent = r_ * (-3 * cos(3*theta));
            double tangent_mag = sqrt(vx_tangent*vx_tangent + vy_tangent*vy_tangent + vz_tangent*vz_tangent);
            double dtheta = (tangent_mag > 0) ? (v * dt_) / tangent_mag : 0;
            theta += dtheta;

            goals.push_back(createTrefoilGoal(v, accel_, theta));
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
            double vx_tangent = r_ * (cos(theta) + 4 * cos(2*theta));
            double vy_tangent = r_ * (-sin(theta) + 4 * sin(2*theta));
            double vz_tangent = r_ * (-3 * cos(3*theta));
            double tangent_mag = sqrt(vx_tangent*vx_tangent + vy_tangent*vy_tangent + vz_tangent*vz_tangent);
            double dtheta = (tangent_mag > 0) ? (v * dt_) / tangent_mag : 0;
            theta += dtheta;

            goals.push_back(createTrefoilGoal(v, 0, theta));
            current_t_traj_ += dt_;
        }
    }
    
    // now decelerate to 0
    index_msgs[goals.size() - 1] = "Trefoil traj: decelerating to 0 m/s";
    while(v > 0){
        // generate points in the Trefoil with *decreasing* velocity
        v = std::max(v - accel_*dt_, 0.0);
        
        // Use consistent theta advancement
        double vx_tangent = r_ * (cos(theta) + 4 * cos(2*theta));
        double vy_tangent = r_ * (-sin(theta) + 4 * sin(2*theta));
        double vz_tangent = r_ * (-3 * cos(3*theta));
        double tangent_mag = sqrt(vx_tangent*vx_tangent + vy_tangent*vy_tangent + vz_tangent*vz_tangent);
        double dtheta = (tangent_mag > 0) ? (v * dt_) / tangent_mag : 0;
        theta += dtheta;

        goals.push_back(createTrefoilGoal(v, -accel_, theta));
    }

    // we reached zero velocity
    if(fabs(v) > 0.001){
        RCLCPP_ERROR(logger_, "Error: final velocity is not zero");
        exit(1);
    }
    index_msgs[goals.size() - 1] = "Trefoil traj: stopped";

    RCLCPP_INFO(logger_, "Time to calculate the traj (s): %f", (clock->now() - tstart).seconds());
    RCLCPP_INFO(logger_, "Goal vector size = %lu", goals.size());
}


snapstack_msgs2::msg::Goal Trefoil::createTrefoilGoal(double v, double accel, double theta) const{
    double t = theta;

    snapstack_msgs2::msg::Goal goal;
    goal.header.frame_id = "world";
    
    // Position (parametric trefoil knot equations)
    goal.p.x = cx_ + r_ * (sin(t) + 2 * sin(2*t));
    goal.p.y = cy_ + r_ * (cos(t) - 2 * cos(2*t));
    goal.p.z = alt_ + r_ * (-sin(3*t));
    
    // Velocity tangent vector (derivatives of position with respect to parameter t)
    double vx_tangent = r_ * (cos(t) + 4 * cos(2*t));
    double vy_tangent = r_ * (-sin(t) + 4 * sin(2*t));
    double vz_tangent = r_ * (-3 * cos(3*t));  // Fixed: added r_ scaling
    
    double tangent_mag = sqrt(vx_tangent*vx_tangent + vy_tangent*vy_tangent + vz_tangent*vz_tangent);
    
    // Unit tangent vector
    double unit_tangent_x = (tangent_mag > 0) ? vx_tangent / tangent_mag : 0;
    double unit_tangent_y = (tangent_mag > 0) ? vy_tangent / tangent_mag : 0;
    double unit_tangent_z = (tangent_mag > 0) ? vz_tangent / tangent_mag : 0;
    
    // Actual velocity (speed * unit tangent)
    goal.v.x = v * unit_tangent_x;
    goal.v.y = v * unit_tangent_y;
    goal.v.z = v * unit_tangent_z;
    
    // Acceleration calculation
    // For parametric curve r(t), acceleration has two components:
    // a = (d²r/dt²) * (dt/ds)² + (dr/dt) * (d²t/ds²)
    // where s is arc length and ds/dt = tangent_mag
    
    // Second derivatives of position with respect to parameter t
    double ax_param = r_ * (-sin(t) - 8 * sin(2*t));
    double ay_param = r_ * (-cos(t) + 8 * cos(2*t));
    double az_param = r_ * (9 * sin(3*t));
    
    // For constant speed motion along curve: d²t/ds² = -curvature/tangent_mag
    // For simplicity, we'll use the tangential and normal components
    if (tangent_mag > 0) {
        double dtds = 1.0 / tangent_mag;  // dt/ds
        
        // Tangential acceleration component (due to speed change)
        double a_tangential = accel;
        
        // Normal acceleration component (centripetal acceleration)
        double curvature_term = (ax_param * unit_tangent_x + ay_param * unit_tangent_y + az_param * unit_tangent_z) * dtds * dtds;
        
        // Total acceleration
        goal.a.x = a_tangential * unit_tangent_x + (ax_param * dtds * dtds - curvature_term * unit_tangent_x);
        goal.a.y = a_tangential * unit_tangent_y + (ay_param * dtds * dtds - curvature_term * unit_tangent_y);
        goal.a.z = a_tangential * unit_tangent_z + (az_param * dtds * dtds - curvature_term * unit_tangent_z);
    } else {
        goal.a.x = 0;
        goal.a.y = 0;
        goal.a.z = 0;
    }
    
    // Jerk (set to zero for simplicity)
    goal.j.x = 0;
    goal.j.y = 0;
    goal.j.z = 0;
    
    // Yaw angle (face direction of velocity)
    goal.psi = atan2(goal.v.y, goal.v.x);
    goal.dpsi = 0;  // dyaw has to be in world frame, the outer loop already converts it to body
    goal.power = true;

    return goal;
}

void Trefoil::generateStopTraj(std::vector<snapstack_msgs2::msg::Goal>& goals,
                              std::unordered_map<int,std::string>& index_msgs,
                              int& pub_index,
                              const rclcpp::Clock::SharedPtr& clock){

    rclcpp::Time tstart = clock->now();

    // Fixed: Calculate 3D velocity magnitude
    double v = sqrt(pow(goals[pub_index].v.x, 2) + 
                    pow(goals[pub_index].v.y, 2) + 
                    pow(goals[pub_index].v.z, 2));
    
    // Calculate current theta from position
    double dx = goals[pub_index].p.x - cx_;
    double dy = goals[pub_index].p.y - cy_;
    double dz = goals[pub_index].p.z - alt_;
    
    // For trefoil knot, we need to solve for theta from the parametric equations
    // This is an approximation - for more accuracy, you'd need numerical methods
    double theta = atan2(dy, dx);  // Rough approximation
    
    std::vector<snapstack_msgs2::msg::Goal> goals_tmp;
    std::unordered_map<int,std::string> index_msgs_tmp;

    index_msgs_tmp[0] = "Trefoil traj: pressed END, decelerating to 0 m/s";

    while(v > 0){
        // generate points in the Trefoil with *decreasing* velocity
        v = std::max(v - accel_*dt_, 0.0);
        
        // Use consistent theta advancement
        double vx_tangent = r_ * (cos(theta) + 4 * cos(2*theta));
        double vy_tangent = r_ * (-sin(theta) + 4 * sin(2*theta));
        double vz_tangent = r_ * (-3 * cos(3*theta));
        double tangent_mag = sqrt(vx_tangent*vx_tangent + vy_tangent*vy_tangent + vz_tangent*vz_tangent);
        double dtheta = (tangent_mag > 0) ? (v * dt_) / tangent_mag : 0;
        theta += dtheta;

        goals_tmp.push_back(createTrefoilGoal(v, -accel_, theta));
    }

    // we reached zero velocity
    index_msgs_tmp[goals_tmp.size() - 1] = "Trefoil traj: stopped";

    goals = std::move(goals_tmp);
    index_msgs = std::move(index_msgs_tmp);
    pub_index = 0;

    RCLCPP_INFO(logger_, "Time to calculate the braking traj (s): %f", (clock->now() - tstart).seconds());
    RCLCPP_INFO(logger_, "Goal vector size = %lu", goals.size());
}

bool Trefoil::trajectoryInsideBounds(double xmin, double xmax,
                                    double ymin, double ymax,
                                    double zmin, double zmax){
    // Fixed: More comprehensive bounds checking for trefoil knot
    // The trefoil knot extends roughly from -3r to +3r in x,y and -r to +r in z
    
    double margin = 0.1;  // Small safety margin
    
    // Check multiple points around the trefoil to ensure it's within bounds
    for (int i = 0; i < 36; ++i) {  // Check 36 points around the curve
        double t = (2 * M_PI * i) / 36.0;
        
        double x = cx_ + r_ * (sin(t) + 2 * sin(2*t));
        double y = cy_ + r_ * (cos(t) - 2 * cos(2*t));
        double z = alt_ + r_ * (-sin(3*t));
        
        if (x < xmin + margin || x > xmax - margin ||
            y < ymin + margin || y > ymax - margin ||
            z < zmin + margin || z > zmax - margin) {
            return false;
        }
    }
    
    return true;
}

} /* namespace */