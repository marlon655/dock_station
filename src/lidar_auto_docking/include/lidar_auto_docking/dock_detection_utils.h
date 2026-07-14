/*
 * Copyright 2026
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#ifndef LIDAR_AUTO_DOCKING__DOCK_DETECTION_UTILS_H_
#define LIDAR_AUTO_DOCKING__DOCK_DETECTION_UTILS_H_

#include <cmath>
#include <cstdint>

#include <geometry_msgs/msg/pose.hpp>
#include <lidar_auto_docking_messages/msg/dock_detection.hpp>
#include <std_msgs/msg/header.hpp>

namespace lidar_auto_docking
{

  inline lidar_auto_docking_messages::msg::DockDetection makeDockDetection(
    const std_msgs::msg::Header & header,
    const geometry_msgs::msg::Pose & pose, double width,
    std::uint32_t point_count, double fitness)
  {
    lidar_auto_docking_messages::msg::DockDetection detection;
    detection.header = header;
    detection.detected = true;
    detection.width = width;
    detection.point_count = point_count;
    detection.fitness = fitness;
    detection.pose = pose;
    detection.lidar_distance =
      std::hypot(pose.position.x, pose.position.y);
    return detection;
  }

}  // namespace lidar_auto_docking

#endif  // LIDAR_AUTO_DOCKING__DOCK_DETECTION_UTILS_H_
