/*
 * Copyright 2026
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#include <gtest/gtest.h>

#include <lidar_auto_docking/laser_processor.h>
#include <lidar_auto_docking/scan_rotation.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace {

constexpr double kPi = 3.14159265358979323846;

sensor_msgs::msg::LaserScan makeCircularDockScan() {
  sensor_msgs::msg::LaserScan scan;
  constexpr std::size_t unique_sample_count = 2169;
  constexpr std::size_t sample_count_with_duplicate =
      unique_sample_count + 1;

  scan.header.frame_id = "laser";
  scan.angle_min = 0.0F;
  scan.angle_increment =
      static_cast<float>((2.0 * kPi) / unique_sample_count);
  scan.angle_max = static_cast<float>(2.0 * kPi);
  scan.range_min = 0.05F;
  scan.range_max = 10.0F;
  scan.ranges.assign(sample_count_with_duplicate,
                     std::numeric_limits<float>::infinity());
  scan.intensities.resize(sample_count_with_duplicate);

  for (std::size_t i = 0; i < unique_sample_count; ++i) {
    scan.intensities[i] = static_cast<float>(1000 + i);
  }

  // 180 samples straddling the original final-index -> index-0 seam.
  for (std::size_t i = 2075; i < unique_sample_count; ++i) {
    scan.ranges[i] = 0.9F;
  }
  for (std::size_t i = 0; i <= 85; ++i) {
    scan.ranges[i] = 0.9F;
  }

  // The final sample repeats the physical direction represented by index 0.
  scan.ranges.back() = scan.ranges.front();
  scan.intensities.back() = scan.intensities.front();
  return scan;
}

}  // namespace

TEST(ScanRotation, PreservesGeometryAndMakesSeamObjectContiguous) {
  const sensor_msgs::msg::LaserScan input = makeCircularDockScan();
  const auto result = lidar_auto_docking::rotateLaserScan(input, 180.0);

  ASSERT_TRUE(result.full_circle);
  ASSERT_TRUE(result.duplicate_endpoint_removed);
  ASSERT_EQ(result.scan.ranges.size(), input.ranges.size() - 1);
  ASSERT_EQ(result.scan.intensities.size(), result.scan.ranges.size());
  const std::int64_t expected_shift = static_cast<std::int64_t>(
      std::llround(kPi / static_cast<double>(input.angle_increment)));
  EXPECT_EQ(result.shift_samples, expected_shift);
  EXPECT_FLOAT_EQ(result.scan.angle_increment, input.angle_increment);

  const std::size_t circular_size = result.scan.ranges.size();
  const std::size_t shift = static_cast<std::size_t>(result.shift_samples);
  for (std::size_t new_index = 0; new_index < circular_size; ++new_index) {
    const std::size_t old_index = (new_index + shift) % circular_size;
    EXPECT_FLOAT_EQ(result.scan.ranges[new_index], input.ranges[old_index]);
    EXPECT_FLOAT_EQ(result.scan.intensities[new_index],
                    input.intensities[old_index]);

    if (!std::isfinite(result.scan.ranges[new_index])) {
      continue;
    }

    const double old_angle =
        input.angle_min + old_index * input.angle_increment;
    const double new_angle =
        result.scan.angle_min + new_index * result.scan.angle_increment;
    const double range = result.scan.ranges[new_index];
    EXPECT_NEAR(range * std::cos(new_angle), range * std::cos(old_angle),
                1e-5);
    EXPECT_NEAR(range * std::sin(new_angle), range * std::sin(old_angle),
                1e-5);
  }

  laser_processor::ScanMask mask;
  laser_processor::ScanProcessor processor(result.scan, mask);
  processor.splitConnected(0.05F);
  processor.removeLessThan(5);

  ASSERT_EQ(processor.getClusters().size(), 1U);
  const laser_processor::SampleSet* cluster =
      processor.getClusters().front();
  ASSERT_NE(cluster, nullptr);
  EXPECT_EQ(cluster->size(), 180U);
  const std::size_t expected_first_index =
      (2075 + circular_size - shift) % circular_size;
  const std::size_t expected_last_index =
      (85 + circular_size - shift) % circular_size;
  EXPECT_EQ((*cluster->begin())->index,
            static_cast<int>(expected_first_index));
  EXPECT_EQ((*cluster->rbegin())->index,
            static_cast<int>(expected_last_index));

  const auto* first = *cluster->begin();
  const auto* last = *cluster->rbegin();
  const double width = std::hypot(first->x - last->x, first->y - last->y);
  EXPECT_GT(width, 0.45);
  EXPECT_LT(width, 0.49);
}

TEST(ScanRotation, SupportsNegativeAndZeroRotation) {
  const sensor_msgs::msg::LaserScan input = makeCircularDockScan();

  const auto unchanged = lidar_auto_docking::rotateLaserScan(input, 0.0);
  EXPECT_EQ(unchanged.shift_samples, 0);
  EXPECT_FALSE(unchanged.duplicate_endpoint_removed);
  EXPECT_EQ(unchanged.scan.ranges, input.ranges);
  EXPECT_EQ(unchanged.scan.intensities, input.intensities);
  EXPECT_FLOAT_EQ(unchanged.scan.angle_min, input.angle_min);
  EXPECT_FLOAT_EQ(unchanged.scan.angle_max, input.angle_max);

  const auto negative = lidar_auto_docking::rotateLaserScan(input, -90.0);
  EXPECT_TRUE(negative.full_circle);
  EXPECT_GT(negative.shift_samples, 0);
  EXPECT_LT(static_cast<std::size_t>(negative.shift_samples),
            negative.scan.ranges.size());
  EXPECT_FLOAT_EQ(negative.scan.angle_increment, input.angle_increment);
}

TEST(ScanRotation, LeavesPartialAndEmptyScansSafe) {
  sensor_msgs::msg::LaserScan partial;
  partial.angle_min = -1.0F;
  partial.angle_increment = 0.01F;
  partial.ranges.assign(100, 1.0F);
  partial.intensities.assign(100, 2.0F);

  const auto partial_result =
      lidar_auto_docking::rotateLaserScan(partial, 180.0);
  EXPECT_FALSE(partial_result.full_circle);
  EXPECT_EQ(partial_result.shift_samples, 0);
  EXPECT_EQ(partial_result.scan.ranges, partial.ranges);
  EXPECT_EQ(partial_result.scan.intensities, partial.intensities);

  sensor_msgs::msg::LaserScan empty;
  const auto empty_result =
      lidar_auto_docking::rotateLaserScan(empty, 180.0);
  EXPECT_FALSE(empty_result.full_circle);
  EXPECT_TRUE(empty_result.scan.ranges.empty());
  EXPECT_EQ(empty_result.shift_samples, 0);
}
