// Copyright 2019 Intelligent Robotics Lab
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

#include <string>
#include <iostream>
#include <vector>
#include <memory>

#include "bt_behavior/LedFeedback.hpp"

#include "std_msgs/msg/bool.hpp"
#include "kobuki_ros_interfaces/msg/led.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"

#define OFF_LED_VAL 0
#define GREEN_LED_VAL 1
#define RED_LED_VAL 3

namespace bt_behavior
{

LedFeedback::LedFeedback(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
  config().blackboard->get("node", led_node_);
  led_publisher_ = led_node_->create_publisher<kobuki_ros_interfaces::msg::Led>(
    "/commands/led", 10);
}

BT::NodeStatus
LedFeedback::tick()
{
  std_msgs::msg::Bool state;
  getInput("status", state);

  auto led_message = kobuki_ros_interfaces::msg::Led();
  if (!on_off_) {
    if (state.data){
        led_message.value = GREEN_LED_VAL;
    } else {
        led_message.value = RED_LED_VAL;  
    }
  } else {
    led_message.value = OFF_LED_VAL;
  }
  
  led_publisher_->publish(led_message);
  on_off_ = !on_off_;
  return BT::NodeStatus::SUCCESS;
}

void
LedFeedback::halt()
{
  std::cout << "LedFeedback halt" << std::endl;
}

}  // namespace bt_behavior

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<bt_behavior::LedFeedback>("LedFeedback");
}
