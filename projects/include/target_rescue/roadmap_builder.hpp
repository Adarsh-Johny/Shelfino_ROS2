// #ifndef TARGET_RESCUE_ROADMAP_BUILDER_HPP
// #define TARGET_RESCUE_ROADMAP_BUILDER_HPP

// #include <vector>
// #include <string>
// #include "geometry_msgs/msg/point.hpp"

// struct RoadmapNode
// {
//     geometry_msgs::msg::Point position;
//     std::vector<size_t> neighbors; // Indices of connected nodes
// };

// class RoadmapBuilder
// {
// public:
//     RoadmapBuilder(); // Constructor

//     // Method to build the roadmap
//     void buildRoadmap(const std::vector<geometry_msgs::msg::Point> &obstacles,
//                       const std::vector<geometry_msgs::msg::Point> &borders,
//                       const std::vector<geometry_msgs::msg::Point> &dynamic_obstacles);

//     // Getter for the roadmap
//     const std::vector<RoadmapNode> &getRoadmap() const;

// private:
//     // Helper function to check if a path is collision-free
//     bool isPathCollisionFree(const geometry_msgs::msg::Point &start,
//                              const geometry_msgs::msg::Point &end,
//                              const std::vector<geometry_msgs::msg::Point> &obstacles);

//     // Roadmap storage
//     std::vector<RoadmapNode> roadmap_;

//     // Map parameters
//     std::string map_type_; // Type of map (e.g., hexagon or rectangle)
//     double map_width_;     // Width of the map
//     double map_height_;    // Height of the map
//     double max_x_;         // Maximum x-coordinate
//     double max_y_;         // Maximum y-coordinate
// };

// #endif // TARGET_RESCUE_ROADMAP_BUILDER_HPP
// t