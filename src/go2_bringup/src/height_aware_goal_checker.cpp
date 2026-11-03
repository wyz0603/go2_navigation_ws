#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include "nav2_controller/plugins/simple_goal_checker.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace go2_bringup
{

// Nav2's planar arrival check also needs a floor check for stacked routes.
class HeightAwareGoalChecker : public nav2_controller::SimpleGoalChecker
{
public:
  void initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & plugin_name,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override
  {
    SimpleGoalChecker::initialize(parent, plugin_name, costmap_ros);
    const auto node = parent.lock();
    if (!node) {
      throw std::runtime_error("HeightAwareGoalChecker requires a lifecycle node");
    }
    const auto parameter = plugin_name + ".z_goal_tolerance";
    if (!node->has_parameter(parameter)) {
      node->declare_parameter(parameter, 0.25);
    }
    z_tolerance_ = node->get_parameter(parameter).as_double();
    if (!std::isfinite(z_tolerance_) || z_tolerance_ <= 0.0) {
      throw std::invalid_argument("z_goal_tolerance must be finite and positive");
    }
  }

  bool isGoalReached(
    const geometry_msgs::msg::Pose & query_pose,
    const geometry_msgs::msg::Pose & goal_pose,
    const geometry_msgs::msg::Twist & velocity) override
  {
    const double dz = query_pose.position.z - goal_pose.position.z;
    if (!std::isfinite(dz) || std::abs(dz) > z_tolerance_) {
      // Do not latch SimpleGoalChecker's stateful XY check on another floor.
      SimpleGoalChecker::reset();
      return false;
    }
    return SimpleGoalChecker::isGoalReached(query_pose, goal_pose, velocity);
  }

  bool getTolerances(
    geometry_msgs::msg::Pose & pose_tolerance,
    geometry_msgs::msg::Twist & vel_tolerance) override
  {
    const bool valid = SimpleGoalChecker::getTolerances(pose_tolerance, vel_tolerance);
    pose_tolerance.position.z = z_tolerance_;
    return valid;
  }

private:
  double z_tolerance_{0.25};
};

}  // namespace go2_bringup

PLUGINLIB_EXPORT_CLASS(go2_bringup::HeightAwareGoalChecker, nav2_core::GoalChecker)
