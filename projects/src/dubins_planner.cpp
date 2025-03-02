#include "target_rescue/dubins_planner.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <limits>
#include <tf2/utils.h>

// Constructor
DubinsPlanner::DubinsPlanner(double turning_radius, rclcpp::Logger logger)
    : turning_radius_(turning_radius), logger_(logger) {}

// Plan Dubins
std::vector<geometry_msgs::msg::Pose> DubinsPlanner::planDubinsPath(
    const geometry_msgs::msg::Pose &start,
    const geometry_msgs::msg::Pose &goal,
    const std::vector<geometry_msgs::msg::Point> &dynamic_obstacles)
{
    RCLCPP_INFO(logger_, "Planning Dubins Path from [%.2f, %.2f] to [%.2f, %.2f]...",
                start.position.x, start.position.y, goal.position.x, goal.position.y);

    std::vector<geometry_msgs::msg::Pose> best_path;
    double best_cost = std::numeric_limits<double>::max();

    double start_x = start.position.x;
    double start_y = start.position.y;
    double start_theta = tf2::getYaw(start.orientation);
    double goal_x = goal.position.x;
    double goal_y = goal.position.y;

    std::vector<std::vector<geometry_msgs::msg::Pose>> possible_paths = {
        generateDubinsPath(start_x, start_y, start_theta, goal_x, goal_y, "LSL"),
        generateDubinsPath(start_x, start_y, start_theta, goal_x, goal_y, "LSR"),
        generateDubinsPath(start_x, start_y, start_theta, goal_x, goal_y, "RSL"),
        generateDubinsPath(start_x, start_y, start_theta, goal_x, goal_y, "RSR"),
        generateDubinsPath(start_x, start_y, start_theta, goal_x, goal_y, "RLR"),
        generateDubinsPath(start_x, start_y, start_theta, goal_x, goal_y, "LRL")};

    for (const auto &path : possible_paths)
    {
        double path_length = calculatePathLength(path);
        if (path_length < best_cost && !isCollision(path, dynamic_obstacles))
        {
            best_cost = path_length;
            best_path = path;
            RCLCPP_INFO(logger_, "Selected path type: %s, Length: %.2f", "LSL", path_length);
        }
    }

    return best_path;
}



// Generate Dubins path
std::vector<geometry_msgs::msg::Pose> DubinsPlanner::generateDubinsPath(
    double start_x, double start_y, double start_theta,
    double goal_x, double goal_y, const std::string &path_type)
{
    std::vector<geometry_msgs::msg::Pose> path;
    double step_size = 0.1;
    double current_x = start_x;
    double current_y = start_y;
    double current_theta = start_theta;
    int step = 1;
    while (std::sqrt(std::pow(current_x - goal_x, 2) + std::pow(current_y - goal_y, 2)) > step_size)
    {
        geometry_msgs::msg::Pose pose;
        current_theta = getUpdatedTheta(path_type, current_theta, step_size, turning_radius_);
        current_x += step_size * std::cos(current_theta);
        current_y += step_size * std::sin(current_theta);

        pose.position.x = current_x;
        pose.position.y = current_y;
        pose.orientation = tf2::toMsg(tf2::Quaternion(0, 0, std::sin(current_theta / 2), std::cos(current_theta / 2)));

        RCLCPP_INFO(logger_, "PATHHHHHHH: x: %f -- y: %f Length",pose.position.x, pose.position.y);

        path.push_back(pose);

        step++;

        if(step > 15000){
            break;
        }
    }

    return path;
}

// Get updated theta
double DubinsPlanner::getUpdatedTheta(const std::string &path_type, double current_theta, double step_size, double turning_radius)
{
    if (path_type == "LSL" || path_type == "RLR")
    {
        return current_theta + step_size / turning_radius;
    }
    else if (path_type == "LSR" || path_type == "RSL" || path_type == "LRL")
    {
        return current_theta - step_size / turning_radius;
    }
    else if (path_type == "RSR")
    {
        return current_theta + step_size / turning_radius;
    }
    return current_theta;
}

// Compute path length
double DubinsPlanner::calculatePathLength(const std::vector<geometry_msgs::msg::Pose> &path)
{
    double length = 0.0;
    for (size_t i = 1; i < path.size(); ++i)
    {
        double dx = path[i].position.x - path[i - 1].position.x;
        double dy = path[i].position.y - path[i - 1].position.y;
        length += std::sqrt(dx * dx + dy * dy);
    }
    return length;
}

// Check for collisions
bool DubinsPlanner::isCollision(const std::vector<geometry_msgs::msg::Pose> &path,
                                const std::vector<geometry_msgs::msg::Point> &dynamic_obstacles) const
{
    RCLCPP_INFO(logger_, "Checking for collisions...");

    for (const auto &pose : path)
    {
        for (const auto &obstacle : dynamic_obstacles)
        {
            double distance = std::hypot(obstacle.x - pose.position.x, obstacle.y - pose.position.y);
            if (distance < 1.0)
            {
                return true;
            }
        }
    }
    return false;
}
