#include <limits>
#include <memory>

#include "gtest/gtest.h"
#include "nav2_core/goal_checker.hpp"
#include "pluginlib/class_loader.hpp"

class HeightAwareGoalCheckerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("height_goal_checker_test");
    checker = loader.createSharedInstance("go2_bringup::HeightAwareGoalChecker");
    checker->initialize(node, "goal_checker", nullptr);
    robot.orientation.w = goal.orientation.w = 1.0;
    robot.position.z = 3.35;
    goal.position.z = 6.35;
  }

  void TearDown() override
  {
    checker.reset();
    node.reset();
    rclcpp::shutdown();
  }

  pluginlib::ClassLoader<nav2_core::GoalChecker> loader{"nav2_core", "nav2_core::GoalChecker"};
  std::shared_ptr<nav2_core::GoalChecker> checker;
  rclcpp_lifecycle::LifecycleNode::SharedPtr node;
  geometry_msgs::msg::Pose robot, goal;
  geometry_msgs::msg::Twist velocity;
};

TEST_F(HeightAwareGoalCheckerTest, SameXYOnDifferentFloorCannotFinish)
{
  EXPECT_FALSE(checker->isGoalReached(robot, goal, velocity));
  robot.position.z = 6.33;
  EXPECT_TRUE(checker->isGoalReached(robot, goal, velocity));
}

TEST_F(HeightAwareGoalCheckerTest, WrongFloorCannotLatchPlanarArrival)
{
  EXPECT_FALSE(checker->isGoalReached(robot, goal, velocity));
  robot.position.z = goal.position.z;
  robot.position.x = 1.0;
  EXPECT_FALSE(checker->isGoalReached(robot, goal, velocity));
  robot.position.x = 0.0;
  EXPECT_TRUE(checker->isGoalReached(robot, goal, velocity));
  // A subsequent floor mismatch must also clear any previous planar latch.
  robot.position.z = 3.35;
  EXPECT_FALSE(checker->isGoalReached(robot, goal, velocity));
  robot.position.z = goal.position.z;
  robot.position.x = 1.0;
  EXPECT_FALSE(checker->isGoalReached(robot, goal, velocity));
}

TEST_F(HeightAwareGoalCheckerTest, InvalidHeightNeverFinishes)
{
  robot.position.z = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(checker->isGoalReached(robot, goal, velocity));
  geometry_msgs::msg::Pose tolerance;
  geometry_msgs::msg::Twist speed_tolerance;
  EXPECT_TRUE(checker->getTolerances(tolerance, speed_tolerance));
  EXPECT_DOUBLE_EQ(tolerance.position.z, 0.25);
}
