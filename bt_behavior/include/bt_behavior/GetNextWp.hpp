#ifndef BT_BEHAVIOR__GETNEXTWP_HPP_
#define BT_BEHAVIOR__GETNEXTWP_HPP_

#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

#include "bt_behavior/ctrl_support/BTActionNode.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

namespace bt_behavior
{

class GetNextWp : public BT::ActionNodeBase
{
 public:
  explicit GetNextWp(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  BT::NodeStatus tick();
  void halt();
  static BT::PortsList providedPorts()
  {
    return {
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("waypoint")
    };
  }

 private:
  rclcpp::Node::SharedPtr node_;  // a node to get the wp parameters
  std::vector<std::string> wp_names_;
};

}  // namespace bt_behavior

#endif  // BT_BEHAVIOR__GETNEXTWP_HPP_
