#include "target_rescue/dubins_planner.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <limits>
#include <tf2/utils.h>


/**
 * @brief Constructor to initialize the Dubins Planner
 * @param turning_radius The minimum turning radius of the robot.
 */
DubinsPlanner::DubinsPlanner(double turning_radius) : turning_radius_(turning_radius) {}

/**
 * @brief Plans the best Dubins path between start and goal positions.
 * @param start The starting pose.
 * @param goal The goal pose.
 * @param dynamic_obstacles List of dynamic obstacles to avoid. TODO
 * @return The shortest valid Dubins path as a vector of poses.
 */
std::vector<geometry_msgs::msg::Pose> DubinsPlanner::planDubinsPath(
    const geometry_msgs::msg::Pose &start,
    const geometry_msgs::msg::Pose &goal,
    const std::vector<geometry_msgs::msg::Point> &dynamic_obstacles)
{
    std::vector<geometry_msgs::msg::Pose> best_path;
    double best_cost = std::numeric_limits<double>::max();

    // Extract start and goal positions and orientations
    double start_x = start.position.x;
    double start_y = start.position.y;
    double start_theta = tf2::getYaw(start.orientation);
    double goal_x = goal.position.x;
    double goal_y = goal.position.y;
    double goal_theta = tf2::getYaw(goal.orientation);

    // Compute all six Dubins paths and evaluate them
    std::vector<std::vector<geometry_msgs::msg::Pose>> possible_paths = {
        generateDubinsPath(start_x, start_y, start_theta, goal_x, goal_y, goal_theta, "LSL"),
        generateDubinsPath(start_x, start_y, start_theta, goal_x, goal_y, goal_theta, "LSR"),
        generateDubinsPath(start_x, start_y, start_theta, goal_x, goal_y, goal_theta, "RSL"),
        generateDubinsPath(start_x, start_y, start_theta, goal_x, goal_y, goal_theta, "RSR"),
        generateDubinsPath(start_x, start_y, start_theta, goal_x, goal_y, goal_theta, "RLR"),
        generateDubinsPath(start_x, start_y, start_theta, goal_x, goal_y, goal_theta, "LRL")};

    // Select the shortest valid Dubins path
    for (const auto &path : possible_paths)
    {
        double path_length = calculatePathLength(path);
        if (path_length < best_cost && !isCollision(path, dynamic_obstacles))
        {
            best_cost = path_length;
            best_path = path;
        }
    }

    return best_path;
}

/**
 * @brief Computes a Dubins path of a specific type(LSL,LSR,RSL,RSR,RLR,LRL).
 * @param start_x, start_y, start_theta The starting position and orientation.
 * @param goal_x, goal_y, goal_theta The goal position and orientation.
 * @param path_type The type of Dubins path (LSL, RSR, etc.).
 * @return A vector of poses representing the computed Dubins path.
 */
std::vector<geometry_msgs::msg::Pose> DubinsPlanner::generateDubinsPath(
    double start_x, double start_y, double start_theta,
    double goal_x, double goal_y, double goal_theta, const std::string &path_type)
{
    std::vector<geometry_msgs::msg::Pose> path;
    double step_size = 0.1;
    double current_x = start_x;
    double current_y = start_y;
    double current_theta = start_theta;

    while (std::sqrt(std::pow(current_x - goal_x, 2) + std::pow(current_y - goal_y, 2)) > step_size)
    {
        geometry_msgs::msg::Pose pose;

        // Get updated theta based on Dubins path type
        current_theta = getUpdatedTheta(path_type, current_theta, step_size, turning_radius_);

        // Update position based on new heading
        current_x += step_size * std::cos(current_theta);
        current_y += step_size * std::sin(current_theta);

        // Store pose
        pose.position.x = current_x;
        pose.position.y = current_y;
        pose.orientation = tf2::toMsg(tf2::Quaternion(0, 0, std::sin(current_theta / 2), std::cos(current_theta / 2)));

        path.push_back(pose);
    }

    return path;
}

/// @brief Computes the updated heading (theta) based on the Dubins path type.
/// @param path_type The type of Dubins path (LSL, RSR, etc.).
/// @param current_theta The current heading angle of the robot.
/// @param step_size The incremental step size for updating the heading.
/// @param turning_radius The minimum turning radius of the robot.
/// @return The updated theta after applying the step change.
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
    return current_theta; // No change if unknown path type
}

/**
 * @brief Computes the total length of a given path.
 * @param path The vector of poses representing the path.
 * @return The total path length.
 */
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

/**
 * @brief Checks if a computed path collides with any dynamic obstacles.
 * @param path The vector of poses representing the path.
 * @param dynamic_obstacles A vector of obstacle positions.
 * @return True if a collision is detected, false otherwise.
 */
bool DubinsPlanner::isCollision(const std::vector<geometry_msgs::msg::Pose> &path,
                                const std::vector<geometry_msgs::msg::Point> &dynamic_obstacles) const
{
    for (const auto &pose : path)
    {
        for (const auto &obstacle : dynamic_obstacles)
        {
            double distance = std::hypot(obstacle.x - pose.position.x, obstacle.y - pose.position.y);
            if (distance < 0.5) // Assume obstacle radius of 0.5 meters
            {
                return true;
            }
        }
    }
    return false;
}
