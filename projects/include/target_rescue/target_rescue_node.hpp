#ifndef TARGET_RESCUE_NODE_HPP
#define TARGET_RESCUE_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "target_rescue/dubins_planner.hpp"

using FollowPath = nav2_msgs::action::FollowPath;
using GoalHandleFollowPath = rclcpp_action::ClientGoalHandle<FollowPath>;

class TargetRescueNode : public rclcpp::Node
{
public:
    TargetRescueNode();

private:
    // Path planner
    DubinsPlanner dubins_planner_;

    // Subscribers
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr victims_sub_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr map_borders_sub_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr obstacles_sub_;

    // Publishers
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

    // Action client for FollowPath
    rclcpp_action::Client<FollowPath>::SharedPtr follow_path_client_;

    // Callbacks
    void victimsCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg);
    void obstaclesCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg);
    void mapBordersCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg);

    // Path computation and execution
    void computeAndPublishPath(const geometry_msgs::msg::Point &victim);
    void sendFollowPathRequest(const std::vector<geometry_msgs::msg::PoseStamped> &path);
    void moveToGate(); // New function for navigating to the gate

    void goalResponseCallback(std::shared_future<GoalHandleFollowPath::SharedPtr> future);
    void feedbackCallback(GoalHandleFollowPath::SharedPtr, const std::shared_ptr<const FollowPath::Feedback> feedback);
    void resultCallback(const GoalHandleFollowPath::WrappedResult &result);

    // Robot state
    geometry_msgs::msg::Pose robot_pose_;

    // Score tracking
    int score_;           // Tracks the score based on rescued victims
    int victims_rescued_; // Counter for rescued victims
    int total_victims_;   // Total number of victims detected

    // Gate positions
    std::vector<double> gate_positions_x_;
    std::vector<double> gate_positions_y_;
};

#endif // TARGET_RESCUE_NODE_HPP