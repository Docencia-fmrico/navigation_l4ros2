// Copyright 2021 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef BT_BEHAVIOR__MOVE_HPP_
#define BT_BEHAVIOR__MOVE_HPP_

#include <string>
#include <memory>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "kobuki_ros_interfaces/msg/sound.hpp"
#include "kobuki_ros_interfaces/msg/led.hpp"
#include "std_msgs/msg/bool.hpp"

#include "bt_behavior/ctrl_support/BTActionNode.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/duration.hpp"

namespace bt_behavior
{

class Move : public bt_behavior::BtActionNode<nav2_msgs::action::NavigateToPose>
{
public:
  explicit Move(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  void on_tick() override;
  BT::NodeStatus on_success() override;
  // We'll need also on_aborted() and on_cancelled() to implement the behavior in case of failing

  BT::NodeStatus on_aborted() override;
  BT::NodeStatus on_cancelled() override;
  // BT::NodeStatus abort();

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("goal"),
      BT::OutputPort<std_msgs::msg::Bool>("status")
    };
  }

private:
  std::shared_ptr<rclcpp::Node> sound_node_;
};

}  // namespace bt_behavior
#endif  // BT_BEHAVIOR__MOVE_HPP_
