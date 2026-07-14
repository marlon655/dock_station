/*
 * Copyright 2026
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#include <lidar_auto_docking/scan_rotation.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace lidar_auto_docking {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;

std::int64_t positiveModulo(std::int64_t value, std::int64_t modulus) {
  const std::int64_t remainder = value % modulus;
  return remainder < 0 ? remainder + modulus : remainder;
}

}  // namespace

ScanRotationResult rotateLaserScan(
    const sensor_msgs::msg::LaserScan& input, double rotation_deg) {
  ScanRotationResult result;
  result.scan = input;

  const std::size_t input_size = input.ranges.size();
  const double increment = static_cast<double>(input.angle_increment);
  const double abs_increment = std::abs(increment);
  if (input_size < 2 || !std::isfinite(rotation_deg) ||
      !std::isfinite(increment) || abs_increment == 0.0) {
    return result;
  }

  // A 360-degree scan is commonly encoded in either of two ways:
  // N distinct circular bins, or N samples containing both 0 and 2*pi.
  const double span_between_endpoints =
      static_cast<double>(input_size - 1) * abs_increment;
  const double span_after_last_sample =
      static_cast<double>(input_size) * abs_increment;
  const double angular_tolerance = 0.5 * abs_increment + 1e-9;
  const bool has_duplicate_endpoint =
      std::abs(span_between_endpoints - kTwoPi) <= angular_tolerance;
  const bool has_distinct_circular_bins =
      std::abs(span_after_last_sample - kTwoPi) <= angular_tolerance;

  result.full_circle = has_duplicate_endpoint || has_distinct_circular_bins;
  if (!result.full_circle) {
    return result;
  }

  std::size_t circular_size = input_size;
  if (has_duplicate_endpoint) {
    circular_size--;
  }
  if (circular_size == 0) {
    return result;
  }

  const double normalized_rotation_deg = std::remainder(rotation_deg, 360.0);
  const double rotation_rad = normalized_rotation_deg * kPi / 180.0;
  const std::int64_t raw_shift =
      static_cast<std::int64_t>(std::llround(rotation_rad / increment));
  result.shift_samples =
      positiveModulo(raw_shift, static_cast<std::int64_t>(circular_size));

  if (result.shift_samples == 0) {
    return result;
  }

  // Some 360-degree drivers publish both ends of the angular interval. They
  // are the same physical direction but create N-1 circular intervals. Drop
  // only that redundant endpoint from this private copy before rotating.
  if (has_duplicate_endpoint) {
    result.scan.ranges.pop_back();
    if (result.scan.intensities.size() == input_size) {
      result.scan.intensities.pop_back();
    }
    result.duplicate_endpoint_removed = true;
  }

  const std::size_t shift =
      static_cast<std::size_t>(result.shift_samples);
  std::rotate(result.scan.ranges.begin(), result.scan.ranges.begin() + shift,
              result.scan.ranges.end());

  if (result.scan.intensities.size() == result.scan.ranges.size()) {
    std::rotate(result.scan.intensities.begin(),
                result.scan.intensities.begin() + shift,
                result.scan.intensities.end());
  }

  // Sample j now contains old sample (j + shift) modulo N. Moving angle_min
  // by the same amount preserves x=r*cos(angle) and y=r*sin(angle). angle_max
  // is rebuilt from the resulting vector size; angle_increment is untouched.
  result.scan.angle_min = static_cast<float>(
      static_cast<double>(input.angle_min) + shift * increment);
  result.scan.angle_max = static_cast<float>(
      static_cast<double>(result.scan.angle_min) +
      static_cast<double>(result.scan.ranges.size() - 1) * increment);

  return result;
}

}  // namespace lidar_auto_docking
