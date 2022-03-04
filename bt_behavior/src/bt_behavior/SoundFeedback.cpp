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

#include "bt_behavior/SoundFeedback.hpp"

#include "std_msgs/msg/bool.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "kobuki_ros_interfaces/msg/sound.hpp"
//#include "kobuki_ros_interfaces/msg/led.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"

#define ERROR_SOUND_NUM 5
#define SUCCESS_SOUND_NUM 6
//#define OFF_LED_VAL 0
//#define GREEN_LED_VAL 1
//#define RED_LED_VAL 3

namespace bt_behavior
{

SoundFeedback::SoundFeedback(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
  config().blackboard->get("node", sound_node_);
  sound_publisher_ = sound_node_->create_publisher<kobuki_ros_interfaces::msg::Sound>(
    "/commands/sound", 10);
}

BT::NodeStatus
SoundFeedback::tick()
{
  std_msgs::msg::Bool state;
  getInput("status", state);

  auto sound_message = kobuki_ros_interfaces::msg::Sound();
  if (state.data) {
    sound_message.value = SUCCESS_SOUND_NUM;
  } else {
    sound_message.value = ERROR_SOUND_NUM;
  }
  sound_publisher_->publish(sound_message);

  /*
  auto led_message = kobuki_ros_interfaces::msg::Led();
  led_message.value = OFF_LED_VAL;
  led_publisher_->publish(led_message);
  */

  return BT::NodeStatus::SUCCESS;
}

}  // namespace bt_behavior

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<bt_behavior::SoundFeedback>("SoundFeedback");
}
