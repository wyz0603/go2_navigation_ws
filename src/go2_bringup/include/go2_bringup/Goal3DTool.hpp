#ifndef GO2_BRINGUP__GOAL3D_TOOL_HPP_
#define GO2_BRINGUP__GOAL3D_TOOL_HPP_

#include <memory>

#include <OgreVector.h>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <rviz_common/tool.hpp>
#include <rviz_rendering/viewport_projection_finder.hpp>

namespace rviz_rendering
{
class Arrow;
}

namespace rviz_common
{
namespace properties
{
class StringProperty;
}
}

namespace go2_bringup
{

class Goal3DTool : public rviz_common::Tool
{
  Q_OBJECT

public:
  Goal3DTool();
  ~Goal3DTool() override;

  void onInitialize() override;
  void activate() override;
  void deactivate() override;

  int processMouseEvent(rviz_common::ViewportMouseEvent & event) override;

private Q_SLOTS:
  void updateTopic();

private:
  void publishGoal(double theta);

  std::shared_ptr<rviz_rendering::Arrow> arrow_;

  enum State
  {
    Position,
    Orientation
  };
  State state_;
  double angle_;
  Ogre::Vector3 goal_position_;

  std::shared_ptr<rviz_rendering::ViewportProjectionFinder> projection_finder_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
  rclcpp::Clock::SharedPtr clock_;
  rviz_common::properties::StringProperty * topic_property_;
};

}  // namespace go2_bringup

#endif  // GO2_BRINGUP__GOAL3D_TOOL_HPP_
