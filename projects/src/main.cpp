#include "rclcpp/rclcpp.hpp"
#include "target_rescue/target_rescue_node.hpp"
#include "target_rescue/dubins_planner.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // Initialize Dubins planner with a turning radius of 1.0
    // DubinsPlanner planner(1.0);

    // Example: Define start and goal poses
    // geometry_msgs::msg::Pose start_pose, goal_pose;
    // start_pose.position.x = 0.0;
    // start_pose.position.y = 0.0;
    // start_pose.orientation.w = 1.0; // Facing forward

    // goal_pose.position.x = 5.0;
    // goal_pose.position.y = 5.0;
    // goal_pose.orientation.w = std::cos(M_PI / 4); // 45-degree orientation
    // goal_pose.orientation.z = std::sin(M_PI / 4);

    // // Plan the Dubins path
    // auto path = planner.planDubinsPath(start_pose, goal_pose);

    // // Print the path
    // for (const auto &pose : path)
    // {
    //     RCLCPP_INFO(rclcpp::get_logger("DubinsPlanner"), "Pose: [%.2f, %.2f]", pose.position.x, pose.position.y);
    // }

    // Launch the TargetRescueNode
    rclcpp::spin(std::make_shared<TargetRescueNode>());

    rclcpp::shutdown();
    return 0;
}
