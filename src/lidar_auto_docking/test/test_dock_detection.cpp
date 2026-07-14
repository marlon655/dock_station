/*
 * Copyright 2026
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#include <cmath>
#include <cstdint>

#include "gtest/gtest.h"
#include "lidar_auto_docking/dock_detection_utils.h"
#include "lidar_auto_docking_messages/msg/initdock.hpp"

TEST(DockDetection, PreservesAcceptedDetectionData) {
  std_msgs::msg::Header header;
  header.stamp.sec = 123;
  header.stamp.nanosec = 456789U;
  header.frame_id = "odom";

  geometry_msgs::msg::Pose pose;
  pose.position.x = 3.0;
  pose.position.y = 4.0;
  pose.position.z = 0.25;
  pose.orientation.z = 0.5;
  pose.orientation.w = 0.866025403784;

  constexpr double width = 0.472;
  constexpr std::uint32_t point_count = 198U;
  constexpr double fitness = 0.0037;

  const auto detection = lidar_auto_docking::makeDockDetection(
      header, pose, width, point_count, fitness);

  EXPECT_TRUE(detection.detected);
  EXPECT_EQ(detection.header.stamp.sec, header.stamp.sec);
  EXPECT_EQ(detection.header.stamp.nanosec, header.stamp.nanosec);
  EXPECT_EQ(detection.header.frame_id, "odom");
  EXPECT_DOUBLE_EQ(detection.width, width);
  EXPECT_EQ(detection.point_count, point_count);
  EXPECT_DOUBLE_EQ(detection.fitness, fitness);
  EXPECT_DOUBLE_EQ(detection.pose.position.x, pose.position.x);
  EXPECT_DOUBLE_EQ(detection.pose.position.y, pose.position.y);
  EXPECT_DOUBLE_EQ(detection.pose.position.z, pose.position.z);
  EXPECT_DOUBLE_EQ(detection.pose.orientation.z, pose.orientation.z);
  EXPECT_DOUBLE_EQ(detection.pose.orientation.w, pose.orientation.w);
  EXPECT_DOUBLE_EQ(detection.lidar_distance, 5.0);
}

TEST(DockDetection, ExistingInitdockInterfaceRemainsAvailable) {
  lidar_auto_docking_messages::msg::Initdock init_dock;
  init_dock.x = 1.0;
  init_dock.y = 2.0;
  init_dock.z = 0.25;
  init_dock.w = 0.75;

  EXPECT_DOUBLE_EQ(init_dock.x, 1.0);
  EXPECT_DOUBLE_EQ(init_dock.y, 2.0);
  EXPECT_DOUBLE_EQ(init_dock.z, 0.25);
  EXPECT_DOUBLE_EQ(init_dock.w, 0.75);
}
