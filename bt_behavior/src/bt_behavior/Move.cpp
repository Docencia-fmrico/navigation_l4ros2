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

#include "bt_behavior/Move.hpp"

#include "std_msgs/msg/bool.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "kobuki_ros_interfaces/msg/sound.hpp"
#include "kobuki_ros_interfaces/msg/led.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"


namespace bt_behavior
{

Move::Move(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: bt_behavior::BtActionNode<nav2_msgs::action::NavigateToPose>(xml_tag_name, action_name,
    conf)
{ 
  RCLCPP_INFO(node_->get_logger(), "[MOVE] Init Move\n");
}

void
Move::on_tick()
{
  RCLCPP_INFO(node_->get_logger(), "[MOVE] Ticked Move\n");
  geometry_msgs::msg::PoseStamped goal;
  getInput("goal", goal);
  goal_.pose = goal;
}

BT::NodeStatus
Move::on_success()
{
  RCLCPP_INFO(node_->get_logger(), "navigation Suceeded");

  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus Move::on_aborted()
{
  RCLCPP_INFO(node_->get_logger(), "navigation Aborted");
  
  
  std_msgs::msg::Bool status;
  status.data = false;
  setOutput("status", status);
  
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus Move::on_cancelled()
{
  RCLCPP_INFO(node_->get_logger(), "navigation Cancelled");
  
  
  std_msgs::msg::Bool status;
  status.data = false;
  setOutput("status", status);
  
  return BT::NodeStatus::FAILURE;
}

}  // namespace bt_behavior

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<bt_behavior::Move>(
        name, "navigate_to_pose", config);
    };

  factory.registerBuilder<bt_behavior::Move>(
    "Move", builder);
}
