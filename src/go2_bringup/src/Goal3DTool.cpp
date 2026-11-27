#include "go2_bringup/Goal3DTool.hpp"

#include <cmath>
#include <string>

#include <OgreCamera.h>
#include <OgrePlane.h>
#include <OgreRay.h>
#include <OgreSceneNode.h>

#include <rviz_common/display_context.hpp>
#include <rviz_common/logging.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/viewport_mouse_event.hpp>
#include <rviz_common/interaction/view_picker_iface.hpp>
#include <rviz_common/properties/string_property.hpp>
#include <rviz_rendering/objects/arrow.hpp>
#include <rviz_rendering/render_window.hpp>

namespace go2_bringup
{

Goal3DTool::Goal3DTool()
: rviz_common::Tool(),
  state_(Position),
  angle_(0.0),
  goal_position_(Ogre::Vector3::ZERO)
{
  shortcut_key_ = 'g';

  topic_property_ = new rviz_common::properties::StringProperty(
    "Topic", "/goal_pose",
    "Topic on which the picked 3D goal pose is published.",
    getPropertyContainer(), SLOT(updateTopic()), this);
}

Goal3DTool::~Goal3DTool() = default;

void Goal3DTool::onInitialize()
{
  arrow_ = std::make_shared<rviz_rendering::Arrow>(
    scene_manager_, nullptr, 1.0f, 0.08f, 0.3f, 0.2f);
  arrow_->setColor(0.0f, 1.0f, 0.0f, 1.0f);
  arrow_->getSceneNode()->setVisible(false);

  projection_finder_ = std::make_shared<rviz_rendering::ViewportProjectionFinder>();

  setName("3D Goal Pose");
  updateTopic();
}

void Goal3DTool::updateTopic()
{
  rclcpp::Node::SharedPtr raw_node = context_->getRosNodeAbstraction().lock()->get_raw_node();
  publisher_ = raw_node->create_publisher<geometry_msgs::msg::PoseStamped>(
    topic_property_->getStdString(), 1);
  clock_ = raw_node->get_clock();
}

void Goal3DTool::activate()
{
  setStatus("Click on a 3D traversable cell to set the goal position, drag to set orientation.");
  state_ = Position;
}

void Goal3DTool::deactivate()
{
  if (arrow_) {
    arrow_->getSceneNode()->setVisible(false);
  }
  state_ = Position;
}

int Goal3DTool::processMouseEvent(rviz_common::ViewportMouseEvent & event)
{
  int flags = 0;

  if (event.leftDown()) {
    Ogre::Vector3 picked;
    if (context_->getViewPicker()->get3DPoint(event.panel, event.x, event.y, picked)) {
      goal_position_ = picked;
    } else {
      state_ = Position;
      setStatus("No 3D surface selected. Click directly on a visible traversable voxel.");
      return flags;
    }

    angle_ = 0.0;
    arrow_->setPosition(goal_position_);
    state_ = Orientation;
    flags |= Render;
  } else if (event.type == QEvent::MouseMove && event.left() && state_ == Orientation) {
    auto camera = rviz_rendering::RenderWindowOgreAdapter::getOgreCamera(
      event.panel->getRenderWindow());
    const Ogre::Ray mouse_ray = camera->getCameraToViewportRay(
      static_cast<float>(event.x) / static_cast<float>(event.panel->width()),
      static_cast<float>(event.y) / static_cast<float>(event.panel->height()));

    const Ogre::Plane goal_plane(Ogre::Vector3::UNIT_Z, goal_position_.z);
    const auto intersection = mouse_ray.intersects(goal_plane);
    if (intersection.first) {
      const Ogre::Vector3 current = mouse_ray.getPoint(intersection.second);
      angle_ = std::atan2(current.y - goal_position_.y, current.x - goal_position_.x);

      arrow_->getSceneNode()->setVisible(true);
      arrow_->setOrientation(
        Ogre::Quaternion(Ogre::Radian(angle_), Ogre::Vector3::UNIT_Z) *
        Ogre::Quaternion(Ogre::Radian(-Ogre::Math::HALF_PI), Ogre::Vector3::UNIT_Y));
      flags |= Render;
    }
  } else if (event.leftUp() && state_ == Orientation) {
    publishGoal(angle_);
    arrow_->getSceneNode()->setVisible(false);
    state_ = Position;
    flags |= (Render | Finished);
  }

  return flags;
}

void Goal3DTool::publishGoal(double theta)
{
  geometry_msgs::msg::PoseStamped goal;
  goal.header.frame_id = context_->getFixedFrame().toStdString();
  goal.header.stamp = clock_->now();

  goal.pose.position.x = goal_position_.x;
  goal.pose.position.y = goal_position_.y;
  goal.pose.position.z = goal_position_.z;

  goal.pose.orientation.x = 0.0;
  goal.pose.orientation.y = 0.0;
  goal.pose.orientation.z = std::sin(theta / 2.0);
  goal.pose.orientation.w = std::cos(theta / 2.0);

  RVIZ_COMMON_LOG_INFO_STREAM(
    "Publishing 3D goal on " << topic_property_->getStdString()
    << ": position (" << goal.pose.position.x << ", " << goal.pose.position.y
    << ", " << goal.pose.position.z << "), yaw " << theta
    << ", frame " << goal.header.frame_id);

  publisher_->publish(goal);
}

}  // namespace go2_bringup

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(go2_bringup::Goal3DTool, rviz_common::Tool)
