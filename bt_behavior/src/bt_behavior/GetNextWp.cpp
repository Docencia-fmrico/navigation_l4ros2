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

#include <string>
#include <iostream>
#include <vector>

#include "bt_behavior/GetNextWp.hpp"
#include "bt_behavior/MyCostmap.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"

#include "std_msgs/msg/bool.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/costmap_subscriber.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"


#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;

namespace bt_behavior
{

using namespace std::chrono_literals;

GetNextWp::GetNextWp(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
  

  config().blackboard->get("node", node_);
  costmap_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid> ("/map", rclcpp::QoS(5).transient_local().reliable(), std::bind(&GetNextWp::mapCallback, this, _1));
  node_->declare_parameter("waypoints");
  wp_names_ = node_->get_parameter("waypoints").as_string_array();

  RCLCPP_INFO(
    node_->get_logger(), "GETNEXTWP init\n");
}

void
GetNextWp::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg){
  map_msg_ = msg;
}


bool
GetNextWp::coordsInMap(double x, double y){

  MyCostmap costmap(*map_msg_);

  if (fabs(costmap.getSizeInMetersX()) < fabs(x) || fabs(costmap.getSizeInMetersY()) < fabs(y)){
    // the coords are out of the max size of the map
    return false;
  }
  unsigned int i,j;
  
  costmap.worldToMap(x, y, i, j);

  return costmap.getCost(i, j) == FREE_SPACE; // check if the point to go is completely free
}


void
GetNextWp::halt()
{
  std::cout << "GetNextWp halt" << std::endl;
}

BT::NodeStatus
GetNextWp::tick()
{ 
  RCLCPP_INFO(node_->get_logger(), "Entered tick GETNEXTWP\n");

  if(map_msg_ == nullptr){
    RCLCPP_INFO(node_->get_logger(), "[GETNEXTWP] Map not recevived yet. Returning RUNNING\n");

    return BT::NodeStatus::RUNNING;
  }

  if (wp_names_.empty()) {
    RCLCPP_INFO(
      node_->get_logger(), "GETNEXTWP: All waypoints sent. Returning FAILURE");

    return BT::NodeStatus::FAILURE;
  }

  std::string wp_str = wp_names_.at(0);
  wp_names_.erase(wp_names_.begin());
  RCLCPP_INFO(
    node_->get_logger(), "GETNEXTWP: Next WP to send is %s", wp_str.c_str());

  node_->declare_parameter(wp_str.c_str());
  geometry_msgs::msg::PoseStamped wp;
  std::vector<double> coords = node_->get_parameter(wp_str.c_str()).as_double_array();

  
  std_msgs::msg::Bool status;

  if(! coordsInMap(coords.at(0), coords.at(1))){
    // the coords is out of the map
    //put the ouput port the state = 0
    RCLCPP_INFO(node_->get_logger(), "The coords %.2f,%.2f are not a free space\n", coords.at(0), coords.at(1));
    status.data = false;
    setOutput("status", status);

    return BT::NodeStatus::SUCCESS;
  }

  status.data = true;
  setOutput("status", status);


  RCLCPP_INFO(
    node_->get_logger(), "x: %.2f, y: %.2f", coords.at(0), coords.at(1));

  wp.pose.position.x = coords.at(0);
  wp.pose.position.y = coords.at(1);
  wp.pose.position.z = 0;
  wp.pose.orientation.x = 0.0;
  wp.pose.orientation.y = 0.0;
  wp.pose.orientation.z = 0.0;
  wp.pose.orientation.w = 1.0;

  wp.header.frame_id = "map";
  wp.header.stamp = node_->now();

  setOutput("waypoint", wp);

  RCLCPP_INFO(
    node_->get_logger(), "GETNEXTWP: %s sent. Returning SUCCESS", wp_str.c_str());

  return BT::NodeStatus::SUCCESS;
}

}  // namespace bt_behavior

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<bt_behavior::GetNextWp>("GetNextWp");
}
