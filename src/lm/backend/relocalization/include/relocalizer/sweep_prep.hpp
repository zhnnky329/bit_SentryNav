// Copyright 2026 bw_nav contributors
// SPDX-License-Identifier: Apache-2.0
//
// Reduction of a live surface into the two 2-D slices consumed by the
// relocalizer: a sparse slice for the global search and a denser slice for
// registration.

#ifndef RELOCALIZER__SWEEP_PREP_HPP_
#define RELOCALIZER__SWEEP_PREP_HPP_

#include <cstddef>
#include <vector>

#include <Eigen/Core>

namespace relocalizer
{

struct SweepSettings
{
  double search_floor{0.35};
  double search_ceiling{2.20};
  double core_floor{0.30};
  double core_ceiling{2.30};
  double nearest_reach{0.80};
  double farthest_reach{60.0};
  double search_cell{0.15};
  int search_limit{800};
  double core_cell{0.20};
  int core_limit{1500};
};

struct SweepSlice
{
  std::vector<Eigen::Vector3d> search_points;
  std::vector<Eigen::Vector3d> core_points;
  std::size_t raw_count{0};
  double reach{0.0};

  bool usable() const {return search_points.size() >= 40 && core_points.size() >= 80;}
};

SweepSlice reduce_sweep(const std::vector<Eigen::Vector3d> & surface, const SweepSettings & settings);

}  // namespace relocalizer

#endif  // RELOCALIZER__SWEEP_PREP_HPP_
