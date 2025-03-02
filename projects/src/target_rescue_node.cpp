#include "target_rescue/target_rescue_node.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// Constructor
TargetRescueNode::TargetRescueNode()
    : Node("target_rescue_node"), dubins_planner_(1.0, this->get_logger()) // Turning radius = 1.0
{
    // Declare parameters
    this->declare_parameter<int>("num_victims", 4);
    this->declare_parameter<std::vector<double>>("victim_positions_x", std::vector<double>{});
    this->declare_parameter<std::vector<double>>("victim_positions_y", std::vector<double>{});
    
    // Get parameters
    victim_positions_x_ = this->get_parameter("victim_positions_x").as_double_array();
    victim_positions_y_ = this->get_parameter("victim_positions_y").as_double_array();
    num_victims_ = this->get_parameter("num_victims").as_int();

    // Action client for Nav2 FollowPath
    follow_path_client_ = rclcpp_action::create_client<nav2_msgs::action::FollowPath>(
        this, "/shelfino1/follow_path");

    // Subscribers
    roadmap_sub_ = this->create_subscription<graph_msgs::msg::GeometryGraph>(
        "/roadmap", 10, std::bind(&TargetRescueNode::roadmap_callback, this, std::placeholders::_1));

    victim_sub_ = this->create_subscription<obstacles_msgs::msg::ObstacleArrayMsg>(
        "/victims",qos, std::bind(&TargetRescueNode::victim_callback, this, std::placeholders::_1));

    robot_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/shelfino1/amcl_pose", 10, std::bind(&TargetRescueNode::robot_pose_callback, this, std::placeholders::_1));

    // Publisher
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/shelfino1/plan1", 10);

    // Initialize robot pose
    robot_pose_.position.x = 0.0;
    robot_pose_.position.y = 0.0;
    robot_pose_.orientation.w = 1.0;

    RCLCPP_INFO(this->get_logger(), "Target Rescue Node started.");
}

// Callback for roadmap
void TargetRescueNode::roadmap_callback(const graph_msgs::msg::GeometryGraph::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received roadmap with %lu edges.", msg->edges.size());
    roadmap_ = *msg;
}

// Callback for robot pose
void TargetRescueNode::robot_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    robot_pose_ = msg->pose.pose;
    RCLCPP_INFO(this->get_logger(), "Updated robot pose: (%.2f, %.2f)", 
                robot_pose_.position.x, robot_pose_.position.y);
}

// Callback for victim detection
void TargetRescueNode::victim_callback(const obstacles_msgs::msg::ObstacleArrayMsg::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received %lu victims.", msg->obstacles.size());

    victim_positions_.clear();
    for (const auto &victim : msg->obstacles)
    {
        if (!victim.polygon.points.empty())
        {
            geometry_msgs::msg::Point victim_pos;
            victim_pos.x = victim.polygon.points[0].x;
            victim_pos.y = victim.polygon.points[0].y;
            victim_positions_.push_back(victim_pos);
        }
    }

    RCLCPP_INFO(this->get_logger(), "Planning rescue path...");
    plan_rescue_path();
}

// Plan the rescue path
void TargetRescueNode::plan_rescue_path()
{
    for (const auto &victim : victim_positions_)
    {
        computeAndPublishPath(victim);
    }

    moveToGate();
}

// Compute Dubins path to a victim
void TargetRescueNode::computeAndPublishPath(const geometry_msgs::msg::Point &victim)
{
    RCLCPP_INFO(this->get_logger(), "**** Computing Dubins path to victim at (%.2f, %.2f)...", victim.x, victim.y);

    geometry_msgs::msg::Pose goal_pose;
    goal_pose.position = victim;
    goal_pose.orientation = robot_pose_.orientation;

    std::vector<geometry_msgs::msg::Pose> dubins_path = 
        dubins_planner_.planDubinsPath(robot_pose_, goal_pose, {});

    if (dubins_path.empty())
    {
        RCLCPP_ERROR(this->get_logger(), "****Dubins path computation failed!****");
        return;
    }

    publish_path(dubins_path);
}

// Publish the planned path
void TargetRescueNode::publish_path(const std::vector<geometry_msgs::msg::Pose> &dubins_path)
{
    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = this->now();
    path_msg.header.frame_id = "map";

    for (const auto &pose : dubins_path)
    {
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header = path_msg.header;
        pose_stamped.pose = pose;
        path_msg.poses.push_back(pose_stamped);
    }

    path_pub_->publish(path_msg);
    sendFollowPathRequest(path_msg);
}

// Move to the nearest gate
void TargetRescueNode::moveToGate()
{
    RCLCPP_INFO(this->get_logger(), "********** All victims rescued! Moving to the gate...");
    //TODO:------------------Move to the nearest gate----------------------//
}

// Send path to Nav2 FollowPath action
void TargetRescueNode::sendFollowPathRequest(const nav_msgs::msg::Path &path)
{
    if (!follow_path_client_->wait_for_action_server(std::chrono::seconds(10)))
    {
        RCLCPP_ERROR(this->get_logger(), "FollowPath action server not available.");
        return;
    }

    auto goal_msg = nav2_msgs::action::FollowPath::Goal();
    goal_msg.path = path;
    follow_path_client_->async_send_goal(goal_msg);
}

// Main function
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TargetRescueNode>());
    rclcpp::shutdown();
    return 0;
}
