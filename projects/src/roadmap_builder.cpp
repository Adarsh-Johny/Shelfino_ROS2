#include "target_rescue/roadmap_builder.hpp"
#include "rclcpp/parameter.hpp"
#include <cmath>

// Constructor
RoadmapBuilder::RoadmapBuilder()
{
    // Declare parameters for map configuration
    this->declare_parameter<std::string>("map", "hexagon");
    this->declare_parameter<double>("dx", 10.0);
    this->declare_parameter<double>("dy", 20.0);

    map_type_ = this->get_parameter("map").as_string();
    map_width_ = this->get_parameter("dx").as_double();
    map_height_ = this->get_parameter("dy").as_double();

    // Set up the roadmap dimensions
    if (map_type_ == "rectangle")
    {
        max_x_ = map_width_;
        max_y_ = map_height_;
    }
    else
    {
        max_x_ = 12.0; // Default hexagon size
        max_y_ = 12.0;
    }

    RCLCPP_INFO(rclcpp::get_logger("RoadmapBuilder"), "Initialized roadmap with type: %s, width: %.2f, height: %.2f",
                map_type_.c_str(), max_x_, max_y_);
}

// Build roadmap function
void RoadmapBuilder::buildRoadmap(const std::vector<geometry_msgs::msg::Point> &obstacles,
                                  const std::vector<geometry_msgs::msg::Point> &borders,
                                  const std::vector<geometry_msgs::msg::Point> &dynamic_obstacles)
{
    roadmap_.clear();

    // Merge static and dynamic obstacles
    std::vector<geometry_msgs::msg::Point> all_obstacles = obstacles;
    all_obstacles.insert(all_obstacles.end(), dynamic_obstacles.begin(), dynamic_obstacles.end());

    // Generate roadmap nodes
    double grid_size = 1.0;
    for (double x = 0; x < max_x_; x += grid_size)
    {
        for (double y = 0; y < max_y_; y += grid_size)
        {
            RoadmapNode node;
            node.position.x = x;
            node.position.y = y;

            // Check if node is inside any obstacle
            bool valid = true;
            for (const auto &obstacle : all_obstacles)
            {
                double distance = std::hypot(obstacle.x - x, obstacle.y - y);
                if (distance < grid_size)
                {
                    valid = false;
                    break;
                }
            }

            if (valid)
            {
                roadmap_.push_back(node);
            }
        }
    }

    RCLCPP_INFO(rclcpp::get_logger("RoadmapBuilder"), "Generated %zu roadmap nodes.", roadmap_.size());

    // Connect nodes with collision-free edges
    for (size_t i = 0; i < roadmap_.size(); ++i)
    {
        for (size_t j = i + 1; j < roadmap_.size(); ++j)
        {
            if (isPathCollisionFree(roadmap_[i].position, roadmap_[j].position, all_obstacles))
            {
                roadmap_[i].neighbors.push_back(j);
                roadmap_[j].neighbors.push_back(i);
            }
        }
    }

    RCLCPP_INFO(rclcpp::get_logger("RoadmapBuilder"), "Connected nodes to form roadmap.");
}

// Get roadmap function
const std::vector<RoadmapNode> &RoadmapBuilder::getRoadmap() const
{
    return roadmap_;
}

// Collision-checking function
bool RoadmapBuilder::isPathCollisionFree(const geometry_msgs::msg::Point &start,
                                         const geometry_msgs::msg::Point &end,
                                         const std::vector<geometry_msgs::msg::Point> &obstacles)
{
    int samples = 10;
    for (int i = 0; i <= samples; ++i)
    {
        double t = static_cast<double>(i) / samples;
        double x = start.x + t * (end.x - start.x);
        double y = start.y + t * (end.y - start.y);

        for (const auto &obstacle : obstacles)
        {
            double distance = std::hypot(obstacle.x - x, obstacle.y - y);
            if (distance < 0.5)
            {
                return false;
            }
        }
    }
    return true;
}
