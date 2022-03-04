// Copyright 2022 <@ivrolan>
#ifndef BT_BEHAVIOR__MYCOSTMAP_HPP_
#define BT_BEHAVIOR__MYCOSTMAP_HPP_


#include <string>
#include <iostream>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/costmap_subscriber.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

// from costmap_2d -> https://github.com/ros-planning/navigation2/blob/main/nav2_util/src/costmap.cpp
#define LETHAL_OBSTACLE 254
#define FREE_SPACE 0
#define NO_INFORMATION 255

// from occupancy grid
#define OCC_GRID_OCCUPIED 100 
#define OCC_GRID_FREE 0
#define OCC_GRID_UNKNOWN -1

class MyCostmap : public nav2_costmap_2d::Costmap2D 
{
  public: 
    MyCostmap(const nav_msgs::msg::OccupancyGrid & map);

};

#endif  // BT_BEHAVIOR__MYCOSTMAP_HPP_