#ifndef DUBINS_PLANNER_HPP
#define DUBINS_PLANNER_HPP

#include <vector>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point.hpp"

/**
 * @brief Dubins Path Planner class to compute shortest Dubins paths.
 */
class DubinsPlanner
{
public:
    /**
     * @brief Constructor to initialize the Dubins Planner.
     * @param turning_radius The minimum turning radius of the robot.
     */
    explicit DubinsPlanner(double turning_radius);

    /**
     * @brief Plans the best Dubins path between start and goal positions.
     * @param start The starting pose.
     * @param goal The goal pose.
     * @param dynamic_obstacles List of dynamic obstacles to avoid.
     * @return The shortest valid Dubins path as a vector of poses.
     */
    std::vector<geometry_msgs::msg::Pose> planDubinsPath(
        const geometry_msgs::msg::Pose &start,
        const geometry_msgs::msg::Pose &goal,
        const std::vector<geometry_msgs::msg::Point> &dynamic_obstacles);

private:
    double turning_radius_; ///< Minimum turning radius of the robot.

    /**
     * @brief Computes a Dubins path of a specific type(LSL,LSR,RSL,RSR,RLR,LRL).
     * @param start_x, start_y, start_theta The starting position and orientation.
     * @param goal_x, goal_y, goal_theta The goal position and orientation.
     * @param path_type The type of Dubins path (LSL, RSR, etc.).
     * @return A vector of poses representing the computed Dubins path.
     */
    std::vector<geometry_msgs::msg::Pose> generateDubinsPath(
        double start_x, double start_y, double start_theta,
        double goal_x, double goal_y, double goal_theta, const std::string &path_type);

    /**
     * @brief Computes the total length of a given path.
     * @param path The vector of poses representing the path.
     * @return The total path length.
     */
    double calculatePathLength(const std::vector<geometry_msgs::msg::Pose> &path);

    /**
     * @brief Checks if a computed path collides with any dynamic obstacles.
     * @param path The vector of poses representing the path.
     * @param dynamic_obstacles A vector of obstacle positions.
     * @return True if a collision is detected, false otherwise.
     */
    bool isCollision(const std::vector<geometry_msgs::msg::Pose> &path,
                     const std::vector<geometry_msgs::msg::Point> &dynamic_obstacles) const;

    /**
     * @brief Computes the updated heading (theta) based on the Dubins path type.
     * @param path_type The type of Dubins path (LSL, RSR, etc.).
     * @param current_theta The current heading angle of the robot.
     * @param step_size The incremental step size for updating the heading.
     * @param turning_radius The minimum turning radius of the robot.
     * @return The updated theta after applying the step change.
     */
    double getUpdatedTheta(const std::string &path_type, double current_theta, double step_size, double turning_radius);
};

#endif // DUBINS_PLANNER_HPP
