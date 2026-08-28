/**
 * @file Csv.cpp
 * @brief Replays a pre-generated snapstack_msgs2/Goal trajectory from a CSV file.
 */
#include "trajectory_generator_ros2/trajectories/Csv.hpp"

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>

namespace trajectory_generator {

namespace {
// Parse a "true"/"false"/"1"/"0" token (case-insensitive) to bool.
bool parseBool(std::string s) {
    for (auto& c : s) c = static_cast<char>(::tolower(c));
    return s == "true" || s == "1";
}

// Expand a leading "~" to $HOME (the shell does not expand it inside a
// `name:=~/path` launch argument, so the param can arrive as a literal "~").
std::string expandUser(const std::string& path) {
    if (path == "~" || path.rfind("~/", 0) == 0) {
        if (const char* home = std::getenv("HOME")) {
            return std::string(home) + path.substr(1);
        }
    }
    return path;
}
} // namespace

Csv::Csv(const std::string& csv_path, double stop_accel, double dt)
    : Trajectory(dt), csv_path_(expandUser(csv_path)), stop_accel_(stop_accel)
{
    // Load eagerly: trajectoryInsideBounds() runs before generateTraj().
    loaded_ = loadCsv(csv_path_);
}

Csv::~Csv() {}

bool Csv::loadCsv(const std::string& path)
{
    std::ifstream file(path);
    if (!file.is_open()) {
        RCLCPP_ERROR(logger_, "Could not open CSV: %s", path.c_str());
        return false;
    }

    std::string line;
    bool header_skipped = false;
    double first_t = 0.0, second_t = 0.0;
    int t_count = 0;

    while (std::getline(file, line)) {
        if (line.empty()) continue;
        if (!header_skipped) { header_skipped = true; continue; }  // skip header row

        std::stringstream ss(line);
        std::string cell;
        std::vector<std::string> cols;
        while (std::getline(ss, cell, ',')) cols.push_back(cell);

        if (cols.size() < 18) {
            RCLCPP_WARN(logger_, "Skipping malformed CSV row (%zu cols)", cols.size());
            continue;
        }

        const double t = std::stod(cols[0]);
        if (t_count == 0) first_t = t;
        else if (t_count == 1) second_t = t;
        ++t_count;

        snapstack_msgs2::msg::Goal g;
        g.p.x = std::stod(cols[1]);  g.p.y = std::stod(cols[2]);  g.p.z = std::stod(cols[3]);
        g.v.x = std::stod(cols[4]);  g.v.y = std::stod(cols[5]);  g.v.z = std::stod(cols[6]);
        g.a.x = std::stod(cols[7]);  g.a.y = std::stod(cols[8]);  g.a.z = std::stod(cols[9]);
        g.j.x = std::stod(cols[10]); g.j.y = std::stod(cols[11]); g.j.z = std::stod(cols[12]);
        g.psi  = std::stod(cols[13]);
        g.dpsi = std::stod(cols[14]);
        g.power = parseBool(cols[15]);
        g.mode_xy = static_cast<uint8_t>(std::stoi(cols[16]));
        g.mode_z  = static_cast<uint8_t>(std::stoi(cols[17]));

        csv_goals_.push_back(std::move(g));
    }

    if (csv_goals_.empty()) {
        RCLCPP_ERROR(logger_, "CSV contained no data rows: %s", path.c_str());
        return false;
    }
    if (t_count >= 2 && second_t > first_t) csv_dt_ = second_t - first_t;

    RCLCPP_INFO(logger_, "Loaded %zu goals from %s (csv dt=%.4f s, duration=%.2f s).",
                csv_goals_.size(), path.c_str(), csv_dt_, csv_goals_.size() * dt_);
    if (csv_dt_ > 0.0 && std::abs(csv_dt_ - dt_) > 1e-4) {
        RCLCPP_WARN(logger_,
                    "CSV sample period (%.4f s) != publish period dt (%.4f s); playback will be "
                    "time-scaled. Set pub_freq to %.1f Hz to play in real time.",
                    csv_dt_, dt_, 1.0 / csv_dt_);
    }
    return true;
}

void Csv::generateTraj(std::vector<snapstack_msgs2::msg::Goal>& goals,
                       std::unordered_map<int,std::string>& index_msgs,
                       const rclcpp::Clock::SharedPtr& /*clock*/)
{
    goals = csv_goals_;
    if (!goals.empty()) {
        index_msgs[0] = "CSV traj: start";
        index_msgs[static_cast<int>(goals.size()) - 1] = "CSV traj: reached end";
    }
}

void Csv::generateStopTraj(std::vector<snapstack_msgs2::msg::Goal>& goals,
                           std::unordered_map<int,std::string>& index_msgs,
                           int& pub_index,
                           const rclcpp::Clock::SharedPtr& /*clock*/)
{
    // Decelerate the current 3D velocity to zero at stop_accel_, integrating
    // position, then hold. Mirrors Line::generateStopTraj generalized to 3D.
    const snapstack_msgs2::msg::Goal& cur = goals[pub_index];
    double vx = cur.v.x, vy = cur.v.y, vz = cur.v.z;
    double speed = std::sqrt(vx*vx + vy*vy + vz*vz);

    std::vector<snapstack_msgs2::msg::Goal> out;
    std::unordered_map<int,std::string> msgs;
    msgs[0] = "CSV traj: pressed END, decelerating to 0 m/s";

    double px = cur.p.x, py = cur.p.y, pz = cur.p.z;
    const double eps = 1e-6;
    double dirx = 0, diry = 0, dirz = 0;
    if (speed > eps) { dirx = vx/speed; diry = vy/speed; dirz = vz/speed; }

    while (speed > 0.0) {
        speed = std::max(speed - stop_accel_ * dt_, 0.0);
        px += dirx * speed * dt_;
        py += diry * speed * dt_;
        pz += dirz * speed * dt_;

        snapstack_msgs2::msg::Goal g;
        g.p.x = px; g.p.y = py; g.p.z = pz;
        g.v.x = dirx * speed; g.v.y = diry * speed; g.v.z = dirz * speed;
        g.a.x = -dirx * stop_accel_; g.a.y = -diry * stop_accel_; g.a.z = -dirz * stop_accel_;
        g.psi = cur.psi; g.dpsi = 0.0;
        g.power = true;
        g.mode_xy = cur.mode_xy; g.mode_z = cur.mode_z;
        out.push_back(g);
        if (speed <= 0.0) break;
    }

    if (out.empty()) {  // already at rest: emit a single hover hold
        snapstack_msgs2::msg::Goal g;
        g.p = cur.p; g.psi = cur.psi; g.power = true;
        g.mode_xy = cur.mode_xy; g.mode_z = cur.mode_z;
        out.push_back(g);
    }
    msgs[static_cast<int>(out.size()) - 1] = "CSV traj: stopped";

    goals = std::move(out);
    index_msgs = std::move(msgs);
    pub_index = 0;
    RCLCPP_INFO(logger_, "Stop trajectory: %zu goals.", goals.size());
}

bool Csv::trajectoryInsideBounds(double xmin, double xmax,
                                 double ymin, double ymax,
                                 double zmin, double zmax)
{
    if (!loaded_) return false;  // CSV failed to load — refuse to fly
    for (const auto& g : csv_goals_) {
        if (!isPointInsideBounds(xmin, xmax, ymin, ymax, zmin, zmax,
                                 Eigen::Vector3d(g.p.x, g.p.y, g.p.z))) {
            RCLCPP_ERROR(logger_,
                         "CSV waypoint (%.2f, %.2f, %.2f) is outside the room bounds.",
                         g.p.x, g.p.y, g.p.z);
            return false;
        }
    }
    return true;
}

} /* namespace */
