#ifndef TARGET_RESCUE_MAP_GENERATOR_HPP_
#define TARGET_RESCUE_MAP_GENERATOR_HPP_

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include "graph_msgs/msg/geometry_graph.hpp"
#include <obstacles_msgs/msg/obstacle_array_msg.hpp>
#include <vector>
#include <random>

class MapGenerator : public rclcpp::Node
{
public:
    MapGenerator();

private:
    // Subscribers
    rclcpp::Subscription<geometry_msgs::msg::Polygon>::SharedPtr subscription_borders_;
    rclcpp::Subscription<obstacles_msgs::msg::ObstacleArrayMsg>::SharedPtr subscription_obstacles_;

    // Publisher
    rclcpp::Publisher<graph_msgs::msg::GeometryGraph>::SharedPtr roadmap_publisher_;

    // Stored map data
    geometry_msgs::msg::Polygon map_borders_;
    obstacles_msgs::msg::ObstacleArrayMsg obstacles_;

    // Callbacks
    void callback_borders(const geometry_msgs::msg::Polygon::SharedPtr msg);
    void callback_obstacles(const obstacles_msgs::msg::ObstacleArrayMsg::SharedPtr msg);

    // Roadmap generation
    void generatePRM();
    void generateRRT();

    // Utility functions
    bool isColliding(const geometry_msgs::msg::Point &point);
    geometry_msgs::msg::Point steer(const geometry_msgs::msg::Point &from, const geometry_msgs::msg::Point &to, double step_size);
    int getNearestNodeIndex(const std::vector<geometry_msgs::msg::Point> &nodes, const geometry_msgs::msg::Point &random_point);
    double randomDouble(double min, double max);
    double distance(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b);
};

#endif // TARGET_RESCUE_MAP_GENERATOR_HPP_
