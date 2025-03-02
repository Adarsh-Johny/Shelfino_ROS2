#ifndef TARGET_RESCUE_NODE_HPP_
#define TARGET_RESCUE_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "graph_msgs/msg/geometry_graph.hpp"
#include "obstacles_msgs/msg/obstacle_array_msg.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "target_rescue/dubins_planner.hpp"

class TargetRescueNode : public rclcpp::Node
{
public:
    TargetRescueNode();

private:
    // Subscribers
    rclcpp::Subscription<graph_msgs::msg::GeometryGraph>::SharedPtr roadmap_sub_;

    rclcpp::Subscription<obstacles_msgs::msg::ObstacleArrayMsg>::SharedPtr victim_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr robot_pose_sub_;

    // Publisher
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

    // Action client
    rclcpp_action::Client<nav2_msgs::action::FollowPath>::SharedPtr follow_path_client_;

    // Stored data
    graph_msgs::msg::GeometryGraph roadmap_;

    std::vector<geometry_msgs::msg::Point> victim_positions_;
    geometry_msgs::msg::Pose robot_pose_;

    // Parameters
    std::vector<double> victim_positions_x_;
    std::vector<double> victim_positions_y_;
    std::vector<double> gate_positions_x_;
    std::vector<double> gate_positions_y_;
    int num_victims_;

    // Dubins planner
    DubinsPlanner dubins_planner_;
    const rmw_qos_profile_t rmw_qos_profile_custom =
    {
        RMW_QOS_POLICY_HISTORY_KEEP_LAST,
        100,
        RMW_QOS_POLICY_RELIABILITY_RELIABLE,
        RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
        RMW_QOS_DEADLINE_DEFAULT,
        RMW_QOS_LIFESPAN_DEFAULT,
        RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
        RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
        false
    };
    rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_custom);


    // Callbacks
    void roadmap_callback(const graph_msgs::msg::GeometryGraph::SharedPtr msg);

    void victim_callback(const obstacles_msgs::msg::ObstacleArrayMsg::SharedPtr msg);
    void robot_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

    // Path planning
    void plan_rescue_path();
    void computeAndPublishPath(const geometry_msgs::msg::Point &victim);
    void publish_path(const std::vector<geometry_msgs::msg::Pose> &dubins_path);
    void moveToGate();
    void sendFollowPathRequest(const nav_msgs::msg::Path &path);
};

#endif // TARGET_RESCUE_NODE_HPP_
