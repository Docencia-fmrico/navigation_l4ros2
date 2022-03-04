// Copyright 2022 <@ivrolan>
#include <string>
#include <iostream>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/costmap_subscriber.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include "bt_behavior/MyCostmap.hpp"


MyCostmap::MyCostmap(const nav_msgs::msg::OccupancyGrid & map) {
  // modified from https://github.com/ros-planning/navigation2/blob/main/nav2_costmap_2d/src/costmap_2d.cpp

  // fill local variables
  size_x_ = map.info.width;
  size_y_ = map.info.height;
  resolution_ = map.info.resolution;
  origin_x_ = map.info.origin.position.x;
  origin_y_ = map.info.origin.position.y;

  // create the costmap
  costmap_ = new unsigned char[size_x_ * size_y_];

  // fill the costmap with a data
  int8_t data;
  for (unsigned int it = 0; it < size_x_ * size_y_; it++) {
    data = map.data[it];
    if (data == OCC_GRID_UNKNOWN) {
      costmap_[it] = NO_INFORMATION;
    } else {
      // Linear conversion from OccupancyGrid data range [OCC_GRID_FREE..OCC_GRID_OCCUPIED]
      // to costmap data range [FREE_SPACE..LETHAL_OBSTACLE]
      costmap_[it] = std::round(
        static_cast<double>(data) * (LETHAL_OBSTACLE - FREE_SPACE) /
        (OCC_GRID_OCCUPIED - OCC_GRID_FREE));
    }
  }
}
