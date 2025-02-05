// #include <gtest/gtest.h>
// #include "target_rescue/dubins_planner.hpp"
// #include "geometry_msgs/msg/pose.hpp"

// // Helper function to create a Pose
// geometry_msgs::msg::Pose createPose(double x, double y, double yaw)
// {
//     geometry_msgs::msg::Pose pose;
//     pose.position.x = x;
//     pose.position.y = y;
//     tf2::Quaternion q;
//     q.setRPY(0, 0, yaw);
//     pose.orientation = tf2::toMsg(q);
//     return pose;
// }

// // Test: Basic Dubins Path Calculation
// TEST(DubinsPlannerTest, BasicPath)
// {
//     DubinsPlanner planner(1.0); // Turning radius of 1.0
//     geometry_msgs::msg::Pose start = createPose(0.0, 0.0, 0.0);
//     geometry_msgs::msg::Pose goal = createPose(5.0, 5.0, M_PI / 4); // 45-degree target

//     auto path = planner.planDubinsPath(start, goal);

//     ASSERT_FALSE(path.empty()) << "Dubins path should not be empty.";
//     EXPECT_NEAR(path.front().position.x, start.position.x, 0.01);
//     EXPECT_NEAR(path.front().position.y, start.position.y, 0.01);
//     EXPECT_NEAR(path.back().position.x, goal.position.x, 0.01);
//     EXPECT_NEAR(path.back().position.y, goal.position.y, 0.01);
// }

// // Test: Obstacle Avoidance Check
// TEST(DubinsPlannerTest, ObstacleAvoidance)
// {
//     DubinsPlanner planner(1.0);
//     geometry_msgs::msg::Pose start = createPose(0.0, 0.0, 0.0);
//     geometry_msgs::msg::Pose goal = createPose(5.0, 5.0, M_PI / 4);

//     // Place an obstacle in the middle of the path
//     std::vector<geometry_msgs::msg::Point> obstacles;
//     geometry_msgs::msg::Point obs;
//     obs.x = 2.5;
//     obs.y = 2.5;
//     obstacles.push_back(obs);

//     auto path = planner.planDubinsPath(start, goal, obstacles);

//     // Ensure path doesn't go directly through the obstacle
//     for (const auto &pose : path)
//     {
//         double dist = std::hypot(pose.position.x - obs.x, pose.position.y - obs.y);
//         EXPECT_GE(dist, 0.5) << "Path should not pass through obstacle.";
//     }
// }

// // Test: Short Distance Path
// TEST(DubinsPlannerTest, ShortDistance)
// {
//     DubinsPlanner planner(1.0);
//     geometry_msgs::msg::Pose start = createPose(0.0, 0.0, 0.0);
//     geometry_msgs::msg::Pose goal = createPose(0.1, 0.1, 0.0); // Very short distance

//     auto path = planner.planDubinsPath(start, goal);

//     EXPECT_LE(path.size(), 2) << "Short paths should have minimal waypoints.";
// }

// // Test: Edge Case - Same Start and Goal
// TEST(DubinsPlannerTest, NoMovement)
// {
//     DubinsPlanner planner(1.0);
//     geometry_msgs::msg::Pose start = createPose(0.0, 0.0, 0.0);
//     geometry_msgs::msg::Pose goal = createPose(0.0, 0.0, 0.0); // Identical to start

//     auto path = planner.planDubinsPath(start, goal);

//     EXPECT_EQ(path.size(), 1) << "Path should have only one point when start and goal are the same.";
// }

// // Run all tests
// int main(int argc, char **argv)
// {
//     ::testing::InitGoogleTest(&argc, argv);
//     return RUN_ALL_TESTS();
// }
