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

#ifndef BT_BEHAVIOR__LED_FEEDBACK_HPP_
#define BT_BEHAVIOR__LED_FEEDBACK_HPP_

#include <string>
#include <memory>

#include "std_msgs/msg/bool.hpp"
#include "kobuki_ros_interfaces/msg/led.hpp"

#include "bt_behavior/ctrl_support/BTActionNode.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/duration.hpp"

namespace bt_behavior
{

class LedFeedback : public BT::ActionNodeBase
{
public:
  explicit LedFeedback(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  BT::NodeStatus tick();
  void halt();

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std_msgs::msg::Bool>("status")
    };
  }

private:
  std::shared_ptr<rclcpp::Node> led_node_;
  rclcpp::Publisher<kobuki_ros_interfaces::msg::Led>::SharedPtr led_publisher_;
  static bool on_off_;
};

}  // namespace bt_behavior
#endif  // BT_BEHAVIOR__LED_FEEDBACK_HPP_
