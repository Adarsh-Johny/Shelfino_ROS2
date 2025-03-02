#ifndef TARGET_RESCUE_DUBINS_PLANNER_HPP_
#define TARGET_RESCUE_DUBINS_PLANNER_HPP_

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <vector>
#include <string>

class DubinsPlanner
{
public:
    // Constructor
    explicit DubinsPlanner(double turning_radius, rclcpp::Logger logger);

    // Plan Dubins path
    std::vector<geometry_msgs::msg::Pose> planDubinsPath(
        const geometry_msgs::msg::Pose &start,
        const geometry_msgs::msg::Pose &goal,
        const std::vector<geometry_msgs::msg::Point> &dynamic_obstacles);

private:
    // Generate Dubins path
    std::vector<geometry_msgs::msg::Pose> generateDubinsPath(
        double start_x, double start_y, double start_theta,
        double goal_x, double goal_y, const std::string &path_type);

    // Compute the updated heading
    double getUpdatedTheta(const std::string &path_type, double current_theta,
                           double step_size, double turning_radius);

    // Compute path length
    double calculatePathLength(const std::vector<geometry_msgs::msg::Pose> &path);

    // Check for collisions
    bool isCollision(const std::vector<geometry_msgs::msg::Pose> &path,
                     const std::vector<geometry_msgs::msg::Point> &dynamic_obstacles) const;

    double turning_radius_;
    rclcpp::Logger logger_;
};

#endif // TARGET_RESCUE_DUBINS_PLANNER_HPP_
