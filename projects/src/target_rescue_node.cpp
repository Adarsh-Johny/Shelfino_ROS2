#include "target_rescue/target_rescue_node.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "rclcpp/parameter.hpp"
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/follow_path.hpp>

// Constructor
TargetRescueNode::TargetRescueNode()
    : Node("target_rescue_node"), dubins_planner_(1.0) // TODO: Turning Radius given as 1
{
    // Declare parameters
    this->declare_parameter<int>("send_obstacles.n_obstacles", 5);
    this->declare_parameter<bool>("send_obstacles.no_cylinders", true);
    this->declare_parameter<bool>("send_obstacles.no_boxes", false);
    this->declare_parameter<double>("send_obstacles.max_size", 1.0);
    this->declare_parameter<double>("send_obstacles.min_size", 0.5);
    this->declare_parameter<std::vector<double>>("send_obstacles.vect_x", {});
    this->declare_parameter<std::vector<double>>("send_obstacles.vect_y", {});

    this->declare_parameter<std::vector<double>>("send_gates.x", {0.0});
    this->declare_parameter<std::vector<double>>("send_gates.y", {0.0});

    this->declare_parameter<int>("send_victims.n_victims", 4);
    this->declare_parameter<std::vector<double>>("send_victims.vect_x", {});
    this->declare_parameter<std::vector<double>>("send_victims.vect_y", {});

    // Read parameters
    n_obstacles_ = this->get_parameter("send_obstacles.n_obstacles").as_int();
    no_cylinders_ = this->get_parameter("send_obstacles.no_cylinders").as_bool();
    no_boxes_ = this->get_parameter("send_obstacles.no_boxes").as_bool();
    obstacle_positions_x_ = this->get_parameter("send_obstacles.vect_x").as_double_array();
    obstacle_positions_y_ = this->get_parameter("send_obstacles.vect_y").as_double_array();

    gate_positions_x_ = this->get_parameter("send_gates.x").as_double_array();
    gate_positions_y_ = this->get_parameter("send_gates.y").as_double_array();

    n_victims_ = this->get_parameter("send_victims.n_victims").as_int();
    victim_positions_x_ = this->get_parameter("send_victims.vect_x").as_double_array();
    victim_positions_y_ = this->get_parameter("send_victims.vect_y").as_double_array();

    gate_positions_x_ = this->get_parameter("send_gates.x").as_double_array();
    gate_positions_y_ = this->get_parameter("send_gates.y").as_double_array();

    // Set up FollowPath action client
    follow_path_client_ = rclcpp_action::create_client<nav2_msgs::action::FollowPath>(
        this, "/shelfino1/follow_path");

    // Initialize subscribers
    victims_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
        "/victims", 10, std::bind(&TargetRescueNode::victimsCallback, this, std::placeholders::_1));

    map_borders_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
        "/map_borders", 10, std::bind(&TargetRescueNode::mapBordersCallback, this, std::placeholders::_1));

    obstacles_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
        "/obstacles", 10, std::bind(&TargetRescueNode::obstaclesCallback, this, std::placeholders::_1));

    // Path publisher
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/shelfino1/plan1", 10);

    // Robot initial pose
    robot_pose_.position.x = 0.0;
    robot_pose_.position.y = 0.0;
    robot_pose_.orientation.w = 1.0;

    // Initialize score
    score_ = 0;
    victims_rescued_ = 0;

    RCLCPP_INFO(this->get_logger(), "Target Rescue Node started.");
}

// Victims callback
void TargetRescueNode::victimsCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
{
    victim_positions_.clear();
    for (const auto &marker : msg->markers)
    {
        victim_positions_.push_back(marker.pose.position);
    }

    RCLCPP_INFO(this->get_logger(), "Received %zu victims.", victim_positions_.size());
    total_victims_ = victim_positions_.size();

    for (const auto &victim : victim_positions_)
    {
        computeAndPublishPath(victim);
    }
}

// Dynamic obstacle callback
void TargetRescueNode::obstaclesCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
{
    dynamic_obstacles_.clear();
    for (const auto &marker : msg->markers)
    {
        dynamic_obstacles_.push_back(marker.pose.position);
    }
    RCLCPP_INFO(this->get_logger(), "Updated dynamic obstacles.");
}

// Map borders callback
void TargetRescueNode::mapBordersCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
{
    map_borders_.clear();
    for (const auto &marker : msg->markers)
    {
        map_borders_.push_back(marker.pose.position);
    }
    RCLCPP_INFO(this->get_logger(), "Received map borders.");
}
// Compute and publish a Dubins path to a victim
void TargetRescueNode::computeAndPublishPath(const geometry_msgs::msg::Point &victim)
{
    geometry_msgs::msg::Pose goal_pose;
    goal_pose.position = victim;
    goal_pose.orientation = robot_pose_.orientation;

    // Compute Dubins path
    std::vector<geometry_msgs::msg::Pose> dubins_path = dubins_planner_.planDubinsPath(robot_pose_, goal_pose, dynamic_obstacles_);

    // Convert to PoseStamped for FollowPath
    std::vector<geometry_msgs::msg::PoseStamped> path_stamped;
    for (const auto &pose : dubins_path)
    {
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.pose = pose;
        path_stamped.push_back(pose_stamped);
    }

    // Publish the path
    nav_msgs::msg::Path path_msg;

    path_msg.header.frame_id = "map";
    path_msg.header.stamp = this->now();
    path_msg.poses = path_stamped;

    path_pub_->publish(path_msg);


    RCLCPP_INFO(this->get_logger(), "Moving to victim at [%.2f, %.2f].", victim.x, victim.y);

    // Send FollowPath action request
    sendFollowPathRequest(path_msg);
    robot_pose_ = dubins_path.back();

    // Increase score when victim is reached
    victims_rescued_++;
    score_ += 10; // Each victim gives 10 points

    // If all victims are rescued, navigate to the gate
    if (victims_rescued_ == total_victims_)
    {
        moveToGate();
    }
}
// Move to the nearest gate
void TargetRescueNode::moveToGate()
{
    RCLCPP_INFO(this->get_logger(), "All victims rescued! Moving to the gate...");

    // Select the nearest gate
    geometry_msgs::msg::Point nearest_gate;
    double min_distance = std::numeric_limits<double>::max();
    for (size_t i = 0; i < gate_positions_x_.size(); i++)
    {
        double distance = std::hypot(robot_pose_.position.x - gate_positions_x_[i], robot_pose_.position.y - gate_positions_y_[i]);
        if (distance < min_distance)
        {
            min_distance = distance;
            nearest_gate.x = gate_positions_x_[i];
            nearest_gate.y = gate_positions_y_[i];
        }
    }

    computeAndPublishPath(nearest_gate);
}

void TargetRescueNode::feedbackCallback(
    GoalHandleFollowPath::SharedPtr,
    const std::shared_ptr<const FollowPath::Feedback> feedback)
{
    RCLCPP_INFO(this->get_logger(), "FollowPath feedback received: Distance to goal: %.2f meters, Speed: %.2f m/s",
                feedback->distance_to_goal, feedback->speed);
}

void TargetRescueNode::resultCallback(const GoalHandleFollowPath::WrappedResult &result)
{
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
    {
        RCLCPP_INFO(this->get_logger(), "FollowPath action completed successfully!");
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "FollowPath action failed.");
    }
}

void TargetRescueNode::goalResponseCallback(std::shared_ptr<GoalHandleFollowPath> goal_handle)
{
    if (!goal_handle)
    {
        RCLCPP_ERROR(this->get_logger(), "FollowPath action goal was rejected.");
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "FollowPath action goal accepted.");
    }
}

void TargetRescueNode::sendFollowPathRequest(const nav_msgs::msg::Path &path)
{
    RCLCPP_INFO(this->get_logger(), "Sending FollowPath action request...");

    if (!follow_path_client_->wait_for_action_server(std::chrono::seconds(10)))
    {
        RCLCPP_ERROR(this->get_logger(), "FollowPath action server not available after waiting.");
        return;
    }

    // Create a goal message
    auto goal_msg = FollowPath::Goal();
    goal_msg.path = path;
    goal_msg.path.header.frame_id = "map";  // Ensure the correct frame
    goal_msg.path.header.stamp = this->now();

    // Define action options and bind callbacks
    auto send_goal_options = rclcpp_action::Client<FollowPath>::SendGoalOptions();

    send_goal_options.goal_response_callback = 
        [this](std::shared_ptr<GoalHandleFollowPath> goal_handle) {
            goalResponseCallback(goal_handle);
        };

    send_goal_options.feedback_callback = 
        [this](GoalHandleFollowPath::SharedPtr goal_handle, const std::shared_ptr<const FollowPath::Feedback> feedback) {
            feedbackCallback(goal_handle, feedback);
        };

    send_goal_options.result_callback = 
        [this](const GoalHandleFollowPath::WrappedResult &result) {
            resultCallback(result);
        };

    // Send goal
    follow_path_client_->async_send_goal(goal_msg, send_goal_options);
}
