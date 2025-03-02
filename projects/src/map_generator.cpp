#include "target_rescue/map_generator.hpp"
#include <graph_msgs/msg/edges.hpp>
#include <cmath>

// Constructor
MapGenerator::MapGenerator() : Node("map_generator")
{
    // Subscribers
    subscription_borders_ = this->create_subscription<geometry_msgs::msg::Polygon>(
        "/map_borders", 10, std::bind(&MapGenerator::callback_borders, this, std::placeholders::_1));
    
    subscription_obstacles_ = this->create_subscription<obstacles_msgs::msg::ObstacleArrayMsg>(
        "/obstacles", 10, std::bind(&MapGenerator::callback_obstacles, this, std::placeholders::_1));

    // Publisher
    roadmap_publisher_ = this->create_publisher<graph_msgs::msg::GeometryGraph>("/roadmap", 10);



    // Declare parameters
    this->declare_parameter<std::string>("map_planning_algorithm", "PRM"); // Default: PRM
}

// Callback for map borders
void MapGenerator::callback_borders(const geometry_msgs::msg::Polygon::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received map borders.");
    map_borders_ = *msg;
}

// Callback for obstacles and roadmap generation
void MapGenerator::callback_obstacles(const obstacles_msgs::msg::ObstacleArrayMsg::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received obstacles.");
    obstacles_ = *msg;

    std::string algorithm = this->get_parameter("map_planning_algorithm").as_string();
    if (algorithm == "PRM")
    {
        generatePRM();
    }
    else if (algorithm == "RRT")
    {
        generateRRT();
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Invalid map planning algorithm selected.");
    }
}

// Generate roadmap using PRM(Probabilistic Roadmap)
void MapGenerator::generatePRM()
{
    RCLCPP_INFO(this->get_logger(), "Generating roadmap using PRM...");

    int num_samples = 100; // Number of random samples
    double connection_radius = 2.0; // Max connection distance

    std::vector<geometry_msgs::msg::Point> nodes;
    graph_msgs::msg::GeometryGraph roadmap;

    for (int i = 0; i < num_samples; i++)
    {
        geometry_msgs::msg::Point node;
        node.x = randomDouble(-5.0, 5.0);
        node.y = randomDouble(-5.0, 5.0);

        if (!isColliding(node))
        {
            nodes.push_back(node);
            RCLCPP_INFO(this->get_logger(), "Node %d: [%.2f, %.2f]", (int)nodes.size(), node.x, node.y);
        }
    }

    // Connect nodes
    for (size_t i = 0; i < nodes.size(); i++)
    {
        for (size_t j = i + 1; j < nodes.size(); j++)
        {
            if (distance(nodes[i], nodes[j]) < connection_radius)
            {
                graph_msgs::msg::Edges edge;
                edge.node_ids = {static_cast<uint32_t>(i), static_cast<uint32_t>(j)};
                roadmap.edges.push_back(edge);
                RCLCPP_INFO(this->get_logger(), "Edge: [%d] -> [%d]", (int)i, (int)j);
            }
        }
    }

    // Publish roadmap
    roadmap_publisher_->publish(roadmap);
    RCLCPP_INFO(this->get_logger(), "PRM roadmap published with %lu nodes.", nodes.size());
}

int MapGenerator::getNearestNodeIndex(const std::vector<geometry_msgs::msg::Point> &nodes, 
    const geometry_msgs::msg::Point &random_point)
{
    int nearest_index = 0;
    double min_dist = std::numeric_limits<double>::max();

    for (size_t i = 0; i < nodes.size(); i++)
    {
        double d = distance(nodes[i], random_point);
        if (d < min_dist)
        {
            min_dist = d;
            nearest_index = static_cast<int>(i);
        }
    }
    return nearest_index;
}


// Generate roadmap using RRT(Rapidly exploring random tree)
void MapGenerator::generateRRT()
{
    RCLCPP_INFO(this->get_logger(), "Generating roadmap using RRT...");

    int max_iterations = 500;
    double step_size = 1.0;
    geometry_msgs::msg::Point start, goal;
    start.x = -4.0;
    start.y = -4.0;
    goal.x = 4.0;
    goal.y = 4.0;

    std::vector<geometry_msgs::msg::Point> nodes = {start};
    graph_msgs::msg::GeometryGraph roadmap;

    for (int i = 0; i < max_iterations; i++)
    {
        geometry_msgs::msg::Point random_point;
        random_point.x = randomDouble(-5.0, 5.0);
        random_point.y = randomDouble(-5.0, 5.0);

        int nearest_index = getNearestNodeIndex(nodes, random_point);
        geometry_msgs::msg::Point new_node = steer(nodes[nearest_index], random_point, step_size);

        if (!isColliding(new_node))
        {
            nodes.push_back(new_node);
            graph_msgs::msg::Edges edge;
            edge.node_ids = {static_cast<int>(nearest_index), static_cast<int>(nodes.size() - 1)};
            roadmap.edges.push_back(edge);

            if (distance(new_node, goal) < step_size)
            {
                roadmap_publisher_->publish(roadmap);
                RCLCPP_INFO(this->get_logger(), "RRT roadmap published with %lu nodes.", nodes.size());
                return;
            }
        }
    }
}

bool MapGenerator::isColliding(const geometry_msgs::msg::Point &point)
{
    for (const auto &obstacle : obstacles_.obstacles)
    {
        double dx = point.x - obstacle.polygon.points[0].x;
        double dy = point.y - obstacle.polygon.points[0].y;
        if (std::sqrt(dx * dx + dy * dy) < 0.5) // Collision threshold
        {
            return true;
        }
    }
    return false;
}

geometry_msgs::msg::Point MapGenerator::steer(const geometry_msgs::msg::Point &from, const geometry_msgs::msg::Point &to, double step_size)
{
    geometry_msgs::msg::Point new_point;
    double theta = atan2(to.y - from.y, to.x - from.x);
    new_point.x = from.x + step_size * cos(theta);
    new_point.y = from.y + step_size * sin(theta);
    return new_point;
}

double MapGenerator::randomDouble(double min, double max)
{
    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(min, max);
    return dis(gen);
}

double MapGenerator::distance(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b)
{
    return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapGenerator>());
    rclcpp::shutdown();
    return 0;
}
