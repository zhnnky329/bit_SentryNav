// Copyright 2026 bw_nav contributors
// SPDX-License-Identifier: Apache-2.0

#include "relocalizer/sweep_prep.hpp"

#include <algorithm>
#include <cmath>
#include <unordered_set>
#include <vector>

#include "relocalizer/voxel_key.hpp"

namespace relocalizer
{

namespace
{

std::vector<Eigen::Vector3d> thin(
  const std::vector<Eigen::Vector3d> & surface, double floor_height,
  double ceiling_height, double nearest, double farthest, double cell, int limit)
{
  std::vector<Eigen::Vector3d> selected;
  selected.reserve(std::min<std::size_t>(surface.size(), static_cast<std::size_t>(limit) * 2));
  std::unordered_set<CellKey2, CellKey2Hash> visited;
  const double near_squared = nearest * nearest;
  const double far_squared = farthest * farthest;
  const double inverse_cell = 1.0 / cell;

  for (const Eigen::Vector3d & sample : surface) {
    if (sample.z() < floor_height || sample.z() > ceiling_height) {
      continue;
    }
    const double range_squared = sample.x() * sample.x() + sample.y() * sample.y();
    if (range_squared < near_squared || range_squared > far_squared) {
      continue;
    }
    const CellKey2 key{
      static_cast<int32_t>(std::floor(sample.x() * inverse_cell)),
      static_cast<int32_t>(std::floor(sample.y() * inverse_cell))};
    if (!visited.insert(key).second) {
      continue;
    }
    selected.push_back(Eigen::Vector3d(sample.x(), sample.y(), sample.z()));
  }

  if (static_cast<int>(selected.size()) > limit) {
    std::vector<Eigen::Vector3d> reduced;
    reduced.reserve(static_cast<std::size_t>(limit));
    const double stride = static_cast<double>(selected.size()) / static_cast<double>(limit);
    for (int index = 0; index < limit; ++index) {
      const std::size_t position =
        std::min(selected.size() - 1, static_cast<std::size_t>(index * stride));
      reduced.push_back(selected[position]);
    }
    return reduced;
  }
  return selected;
}

}  // namespace

SweepSlice reduce_sweep(const std::vector<Eigen::Vector3d> & surface, const SweepSettings & settings)
{
  SweepSlice slice;
  slice.raw_count = surface.size();
  slice.search_points = thin(
    surface, settings.search_floor, settings.search_ceiling, settings.nearest_reach,
    settings.farthest_reach, settings.search_cell, settings.search_limit);
  slice.core_points = thin(
    surface, settings.core_floor, settings.core_ceiling, settings.nearest_reach,
    settings.farthest_reach, settings.core_cell, settings.core_limit);
  if (slice.search_points.empty()) {
    return slice;
  }

  std::vector<double> ranges;
  ranges.reserve(slice.search_points.size());
  for (const Eigen::Vector3d & point : slice.search_points) {
    ranges.push_back(std::hypot(point.x(), point.y()));
  }
  const std::size_t position = static_cast<std::size_t>(0.95 * (ranges.size() - 1));
  std::nth_element(ranges.begin(), ranges.begin() + position, ranges.end());
  slice.reach = ranges[position];
  return slice;
}

}  // namespace relocalizer
