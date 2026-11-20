// Copyright 2026 bw_nav contributors
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include "relocalizer/surface_atlas.hpp"
#include "relocalizer/plan_atlas.hpp"
#include "relocalizer/placement.hpp"
#include "relocalizer/sweep_prep.hpp"

namespace
{

std::string temporary_path(const std::string & name)
{
  return (std::filesystem::temp_directory_path() / name).string();
}

}  // namespace

TEST(Placement, WrapsAngles)
{
  EXPECT_NEAR(relocalizer::wrap_heading(3.0 * relocalizer::kPi), -relocalizer::kPi, 1.0e-9);
  EXPECT_NEAR(relocalizer::wrap_heading(2.5 * relocalizer::kPi), 0.5 * relocalizer::kPi, 1.0e-9);
  EXPECT_NEAR(relocalizer::wrap_heading(0.25), 0.25, 1.0e-12);
}

TEST(Placement, CompositionMatchesInverse)
{
  relocalizer::Placement outer;
  outer.x = 2.0;
  outer.y = -1.0;
  outer.heading = 0.7;
  relocalizer::Placement inner;
  inner.x = -0.4;
  inner.y = 1.3;
  inner.heading = -1.1;

  const relocalizer::Placement combined = relocalizer::compose(outer, inner);
  const relocalizer::Placement restored = relocalizer::compose(combined, relocalizer::invert(inner));
  EXPECT_NEAR(restored.x, outer.x, 1.0e-9);
  EXPECT_NEAR(restored.y, outer.y, 1.0e-9);
  EXPECT_NEAR(restored.heading, outer.heading, 1.0e-9);
}

TEST(PlanAtlas, BuildsDistanceField)
{
  const int columns = 100;
  const int rows = 100;
  std::vector<uint8_t> mask(static_cast<std::size_t>(columns) * rows, 0);
  for (int row = 0; row < rows; ++row) {
    mask[static_cast<std::size_t>(row) * columns + 50] = 1;
  }
  relocalizer::PlanAtlas plan;
  std::string error;
  ASSERT_TRUE(plan.adopt(columns, rows, 0.1, 0.0, 0.0, 0.0, mask, 0.5, error)) << error;
  EXPECT_NEAR(plan.clearance_at(5.05, 5.0), 0.0, 0.06);
  EXPECT_NEAR(plan.clearance_at(4.05, 5.0), 1.0, 0.11);
  EXPECT_NEAR(plan.affinity_at(4.05, 5.0), std::exp(-2.0), 0.02);
  EXPECT_FALSE(plan.contains(-1.0, 5.0));
  EXPECT_EQ(plan.blocked_at(5.05, 5.0), true);
}

TEST(PlanAtlas, ReadsPgm)
{
  const std::string path = temporary_path("relocalizer_test_map.pgm");
  {
    std::ofstream stream(path, std::ios::binary);
    stream << "P5\n# a comment\n4 3\n255\n";
    const unsigned char payload[12] = {
      0, 0, 0, 0,
      254, 254, 254, 254,
      254, 254, 205, 254};
    stream.write(reinterpret_cast<const char *>(payload), sizeof(payload));
  }
  int columns = 0;
  int rows = 0;
  int maximum = 0;
  std::vector<uint16_t> raster_values;
  std::string error;
  ASSERT_TRUE(relocalizer::PlanAtlas::read_pgm(path, columns, rows, maximum, raster_values, error));
  EXPECT_EQ(columns, 4);
  EXPECT_EQ(rows, 3);
  EXPECT_EQ(maximum, 255);
  ASSERT_EQ(raster_values.size(), 12U);
  EXPECT_EQ(raster_values[4], 254);
  std::filesystem::remove(path);
}

TEST(SurfaceAtlas, ReadsBinaryPcd)
{
  const std::string path = temporary_path("relocalizer_test_surface.pcd");
  {
    std::ofstream stream(path, std::ios::binary);
    stream <<
      "# .PCD v0.7\nVERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\n"
      "TYPE F F F F\nCOUNT 1 1 1 1\nWIDTH 3\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n"
      "POINTS 3\nDATA binary\n";
    const float payload[12] = {
      1.0F, 2.0F, 3.0F, 0.0F,
      -1.0F, 0.5F, 1.5F, 0.0F,
      0.0F, 0.0F, 0.0F, 0.0F};
    stream.write(reinterpret_cast<const char *>(payload), sizeof(payload));
  }
  std::vector<Eigen::Vector3d> points;
  std::string error;
  ASSERT_TRUE(relocalizer::SurfaceAtlas::read_points(path, points, error)) << error;
  ASSERT_EQ(points.size(), 3U);
  EXPECT_NEAR(points[0].x(), 1.0, 1.0e-6);
  EXPECT_NEAR(points[1].y(), 0.5, 1.0e-6);
  std::filesystem::remove(path);
}

TEST(SweepPrep, ReducesSweepToTwoBands)
{
  std::vector<Eigen::Vector3d> surface;
  for (int index = 0; index < 400; ++index) {
    const double angle = 0.01 * index;
    const double radius = 2.0 + 0.01 * index;
    surface.emplace_back(radius * std::cos(angle), radius * std::sin(angle), 1.0);
  }
  relocalizer::SweepSettings settings;
  const relocalizer::SweepSlice slice = relocalizer::reduce_sweep(surface, settings);
  EXPECT_TRUE(slice.usable());
  EXPECT_GT(slice.reach, 2.0);
  EXPECT_LE(slice.search_points.size(), static_cast<std::size_t>(settings.search_limit));
  EXPECT_LE(slice.core_points.size(), static_cast<std::size_t>(settings.core_limit));
}
