/*
 * Copyright 2026
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#ifndef LIDAR_AUTO_DOCKING_SCAN_ROTATION_H
#define LIDAR_AUTO_DOCKING_SCAN_ROTATION_H

#include <cstdint>

#include <sensor_msgs/msg/laser_scan.hpp>

namespace lidar_auto_docking {

struct ScanRotationResult {
  sensor_msgs::msg::LaserScan scan;
  std::int64_t shift_samples{0};
  bool full_circle{false};
  bool duplicate_endpoint_removed{false};
};

/**
 * @brief Rotate a full-circle LaserScan without modifying the input message.
 *
 * Positive rotation moves the logical seam towards increasing scan angles.
 * If the scan contains both endpoints of the circle (for example, 0 and
 * 2*pi), the duplicated final endpoint is removed from the local copy. This
 * makes the number of angular intervals match the number of circular bins and
 * keeps Cartesian coordinates unchanged across the rotated seam.
 *
 * Partial scans are returned unchanged because circular rotation would join
 * physically unrelated directions.
 */
ScanRotationResult rotateLaserScan(
    const sensor_msgs::msg::LaserScan& input, double rotation_deg);

}  // namespace lidar_auto_docking

#endif  // LIDAR_AUTO_DOCKING_SCAN_ROTATION_H
