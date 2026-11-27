// Replan collision-checked SCAN paths from the global route and live lidar.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "planner_interface.h"

class ScanLocalPlanner : public rclcpp::Node {
public:
  ScanLocalPlanner() : Node("scan_local_planner"), buffer_(get_clock()), listener_(buffer_) {
    world_ = declare_parameter<std::string>("world_frame", "world");
    base_ = declare_parameter<std::string>("base_frame", "base");
    planner_.initParam(0.5, 1.0, 2.0);
    lookahead_ = declare_parameter<double>("lookahead_distance", 0.3);
    const double radius = declare_parameter<double>("robot_radius", 0.25);
    const double offset = declare_parameter<double>("footprint_offset", 0.15);
    if (!std::isfinite(radius) || radius <= 0.0 || !std::isfinite(offset) || offset < 0.0 ||
        !std::isfinite(lookahead_) || lookahead_ <= 0.0) {
      throw std::invalid_argument("Require positive robot_radius and nonnegative footprint_offset");
    }
    planner_.initEsdfMap(12.0, 12.0, 4.0, 0.1, Eigen::Vector3d::Zero(), radius, offset);
    output_ = create_publisher<nav_msgs::msg::Path>("/scan/local_path", 1);
    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      "/jie/planned_path", rclcpp::QoS(1).transient_local(),
      [this](nav_msgs::msg::Path::SharedPtr msg) {
        path_ = msg;
        previous_route_.clear();
      });
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/go2/obstacle_points", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        cloud_ = msg;
        cloud_received_ = std::chrono::steady_clock::now();
      });
    status_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/jie/controller_active", rclcpp::QoS(1).transient_local(),
      [this](std_msgs::msg::Bool::SharedPtr msg) {
        executing_ = msg->data;
        if (!executing_) {
          previous_route_.clear();
          clearOutput();
        }
      });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::SharedPtr msg) {
        odom_ = msg;
        const auto &v = msg->twist.twist.linear;
        if (std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z)) {
          // Average gait oscillations before using velocity as a spline boundary.
          velocity_ = 0.9 * velocity_ + 0.1 * Eigen::Vector3d(v.x, v.y, v.z);
        }
        odom_received_ = std::chrono::steady_clock::now();
      });
    timer_ = create_wall_timer(std::chrono::milliseconds(200), [this]() { plan(); });
  }

private:
  void clearOutput(const char *reason = nullptr) {
    if (reason && executing_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "No local path: %s", reason);
    }
    nav_msgs::msg::Path empty;
    empty.header.frame_id = world_;
    empty.header.stamp = now();
    output_->publish(empty);
  }
  void plan() {
    if (!executing_) { clearOutput(); return; }
    if (!path_ || path_->header.frame_id != world_ || path_->poses.empty()) {
      clearOutput("missing or invalid global path"); return;
    }
    if (!cloud_) { clearOutput("missing obstacle cloud"); return; }
    if (!odom_) { clearOutput("missing odometry"); return; }
    if (std::chrono::steady_clock::now() - cloud_received_ > std::chrono::seconds(1)) {
      clearOutput("obstacle cloud reception timed out"); return;
    }
    if (std::chrono::steady_clock::now() - odom_received_ > std::chrono::seconds(1)) {
      clearOutput("odometry reception timed out"); return;
    }
    const double cloud_age = (now() - rclcpp::Time(cloud_->header.stamp)).seconds();
    const double odom_age = (now() - rclcpp::Time(odom_->header.stamp)).seconds();
    if (cloud_age > 0.5 || cloud_age < -0.1) { clearOutput("stale obstacle cloud timestamp"); return; }
    if (odom_age > 0.5 || odom_age < -0.1) { clearOutput("stale odometry timestamp"); return; }
    if (odom_->child_frame_id != base_) { clearOutput("odometry child frame mismatch"); return; }
    try {
      const auto robot = buffer_.lookupTransform(world_, base_, tf2::TimePointZero);
      const auto sensor = buffer_.lookupTransform(world_, cloud_->header.frame_id,
                                                   rclcpp::Time(cloud_->header.stamp));
      const auto &t = robot.transform.translation;
      const auto &q = robot.transform.rotation;
      const auto &v = odom_->twist.twist.linear;
      if (!std::isfinite(v.x) || !std::isfinite(v.y) || !std::isfinite(v.z)) {
        clearOutput("non-finite odometry velocity"); return;
      }
      const tf2::Quaternion rotation(q.x, q.y, q.z, q.w);
      const tf2::Vector3 velocity_world = tf2::quatRotate(rotation, tf2::Vector3(velocity_.x(), velocity_.y(), velocity_.z()));
      scan_planner::PathPoint pose{};
      pose.x = t.x; pose.y = t.y; pose.z = t.z;
      pose.theta = tf2::getYaw(tf2::Quaternion(q.x, q.y, q.z, q.w));
      if (!std::isfinite(pose.x) || !std::isfinite(pose.y) ||
          !std::isfinite(pose.z) || !std::isfinite(pose.theta)) {
        clearOutput("invalid robot pose"); return;
      }
      size_t nearest = 0;
      double nearest_distance = 1e100;
      for (size_t i = 0; i < path_->poses.size(); ++i) {
        const auto &p = path_->poses[i].pose.position;
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
            (!path_->poses[i].header.frame_id.empty() &&
             path_->poses[i].header.frame_id != world_)) {
          clearOutput("invalid global path pose"); return;
        }
        const double dist = std::hypot(std::hypot(p.x - pose.x, p.y - pose.y), p.z - pose.z);
        if (dist < nearest_distance) { nearest_distance = dist; nearest = i; }
      }
      if (nearest_distance > 2.0) { clearOutput("robot too far from global path"); return; }
      // The global planner samples free voxels above the support surface.
      // Keep their height changes while planning at the measured body height.
      const double height_offset = pose.z - path_->poses[nearest].pose.position.z;
      std::vector<scan_planner::PathPoint> points{pose};
      double length = 0.0;
      constexpr double horizon = 3.0;
      const auto &goal = path_->poses.back().pose;
      const double goal_distance = std::hypot(goal.position.x - pose.x, goal.position.y - pose.y);
      const bool near_goal = goal_distance < 0.3 &&
        std::abs(goal.position.z + height_offset - pose.z) < 0.2;
      // Seed with the last accepted route to retain the chosen side of an obstacle.
      // Every candidate is checked again against the current cloud below.
      if (!near_goal && !previous_route_.empty()) {
        size_t closest = 0;
        double distance = 1e100;
        for (size_t i = 0; i < previous_route_.size(); ++i) {
          const auto &p = previous_route_[i];
          const double d = std::hypot(p.x - pose.x, p.y - pose.y);
          if (d < distance) { distance = d; closest = i; }
        }
        if (distance < 0.6) {
          for (size_t i = closest + 1; i < previous_route_.size() && length < 1.8; ++i) {
            const auto &p = previous_route_[i];
            const auto &last = points.back();
            const double d = std::hypot(std::hypot(p.x-last.x, p.y-last.y), p.z-last.z);
            if (d < 0.12) continue;
            points.push_back(p);
            length += d;
          }
          double end_distance = 1e100;
          for (size_t i = nearest; i < path_->poses.size(); ++i) {
            const auto &p = path_->poses[i].pose.position;
            const double d = std::hypot(std::hypot(p.x-points.back().x, p.y-points.back().y),
                                        p.z+height_offset-points.back().z);
            if (d < end_distance) { end_distance = d; nearest = i; }
          }
        }
      }
      for (size_t i = nearest; i < path_->poses.size() && length < horizon; ++i) {
        auto p = path_->poses[i].pose.position;
        p.z += height_offset;
        const auto previous = points.back();
        const double dist = std::hypot(std::hypot(p.x - previous.x, p.y - previous.y), p.z - previous.z);
        if (dist < 0.01) continue;
        const double segment = std::min(dist, horizon - length);
        const int steps = std::max(4, static_cast<int>(std::ceil(segment / 0.2)));
        for (int j = 1; j <= steps; ++j) {
          const double a = (segment / dist) * static_cast<double>(j) / steps;
          scan_planner::PathPoint sample{};
          sample.x = previous.x + a * (p.x - previous.x);
          sample.y = previous.y + a * (p.y - previous.y);
          sample.z = previous.z + a * (p.z - previous.z);
          points.push_back(sample);
        }
        length += segment;
      }
      // Keep enough samples near the final goal for cubic optimization.
      while (points.size() < 5) points.push_back(points.back());
      const auto &st = sensor.transform.translation;
      const auto &sq = sensor.transform.rotation;
      tf2::Transform transform(tf2::Quaternion(sq.x, sq.y, sq.z, sq.w),
                               tf2::Vector3(st.x, st.y, st.z));
      std::vector<scan_planner::ObstacleInfo> obstacles;
      sensor_msgs::PointCloud2ConstIterator<float> x(*cloud_, "x"), y(*cloud_, "y"), z(*cloud_, "z");
      for (; x != x.end(); ++x, ++y, ++z) {
        if (!std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*z)) continue;
        const auto p = transform * tf2::Vector3(*x, *y, *z);
        if (std::abs(p.x() - pose.x) > 5.9 || std::abs(p.y() - pose.y) > 5.9 ||
            std::abs(p.z() - pose.z) > 1.5) continue;
        scan_planner::ObstacleInfo obstacle{};
        obstacle.x = p.x(); obstacle.y = p.y(); obstacle.z = p.z();
        obstacles.push_back(obstacle);
      }
      planner_.setGridMapPos(pose);
      planner_.setVelocity(Eigen::Vector3d(velocity_world.x(), velocity_world.y(), velocity_world.z()));
      planner_.setObstacles(obstacles);
      std::vector<scan_planner::PathPoint> trajectory;
      bool valid = false;
      std::vector<scan_planner::PathPoint> complete_route;
      for (double horizon_length : {3.0, 2.0, 1.2, 0.7}) {
        auto reference = scan_planner::PlannerInterface::trajectoryPrefix(points, horizon_length);
        if (reference.size() < 5) continue;
        planner_.setPathPoint(reference);
        planner_.makePlan();
        planner_.getLocalPlanTrajResults(trajectory);
        complete_route = trajectory;
        // Execute only the near prefix and validate every segment against the
        // full robot footprint; never relax collision or out-of-map checks.
        trajectory = scan_planner::PlannerInterface::trajectoryPrefix(trajectory, 1.5);
        if (planner_.trajectoryIsCollisionFree(trajectory)) {
          valid = true;
          break;
        }
      }
      // Very short final segments can be degenerate for cubic optimization.
      // Validate the straight terminal segment and the complete final rotation.
      if (near_goal) {
        trajectory = points;
        const auto &q_goal = goal.orientation;
        const double goal_yaw = tf2::getYaw(tf2::Quaternion(q_goal.x, q_goal.y, q_goal.z, q_goal.w));
        valid = planner_.trajectoryIsCollisionFree(trajectory) &&
          planner_.rotationIsCollisionFree(pose, pose.theta, goal_yaw) &&
          planner_.rotationIsCollisionFree(trajectory.back(), pose.theta, goal_yaw);
      }
      if (!valid) {
        clearOutput("trajectory rejected or no solution"); return;
      }
      if (!near_goal) {
        const auto tracking_prefix = scan_planner::PlannerInterface::trajectoryPrefix(trajectory, lookahead_);
        const auto &target = tracking_prefix.back();
        const double target_yaw = std::atan2(target.y - pose.y, target.x - pose.x);
        if (!planner_.rotationIsCollisionFree(pose, pose.theta, target_yaw)) {
          clearOutput("initial turn intersects obstacle"); return;
        }
      }
      previous_route_ = complete_route;
      // A result retains the observation time so a slow solve cannot appear fresh.
      nav_msgs::msg::Path output;
      output.header.frame_id = world_; output.header.stamp = cloud_->header.stamp;
      double tangent_yaw = pose.theta;
      for (size_t i = 0; i < trajectory.size(); ++i) {
        const auto &p = trajectory[i];
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
          clearOutput("trajectory has non-finite coordinates"); return;
        }
        geometry_msgs::msg::PoseStamped point;
        point.header = output.header;
        point.pose.position.x = p.x; point.pose.position.y = p.y; point.pose.position.z = p.z;
        const auto &a = trajectory[i == trajectory.size() - 1 ? i - 1 : i];
        const auto &b = trajectory[i == trajectory.size() - 1 ? i : i + 1];
        const double dx = b.x - a.x, dy = b.y - a.y;
        if (std::hypot(dx, dy) > 1e-6) tangent_yaw = std::atan2(dy, dx);
        tf2::Quaternion orientation;
        orientation.setRPY(0.0, 0.0, tangent_yaw);
        point.pose.orientation.x = orientation.x();
        point.pose.orientation.y = orientation.y();
        point.pose.orientation.z = orientation.z();
        point.pose.orientation.w = orientation.w();
        output.poses.push_back(point);
      }
      output_->publish(output);
    } catch (const std::exception &error) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "SCAN input unavailable: %s", error.what());
      clearOutput();
    }
  }
  std::string world_, base_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
  scan_planner::PlannerInterface planner_;
  nav_msgs::msg::Path::SharedPtr path_;
  sensor_msgs::msg::PointCloud2::SharedPtr cloud_;
  nav_msgs::msg::Odometry::SharedPtr odom_;
  bool executing_ = false;
  double lookahead_ = 0.3;
  Eigen::Vector3d velocity_ = Eigen::Vector3d::Zero();
  std::vector<scan_planner::PathPoint> previous_route_;
  std::chrono::steady_clock::time_point cloud_received_{};
  std::chrono::steady_clock::time_point odom_received_{};
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr status_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr output_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScanLocalPlanner>());
  rclcpp::shutdown();
  return 0;
}
