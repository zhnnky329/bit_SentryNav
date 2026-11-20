// Copyright 2026 bw_nav contributors
// SPDX-License-Identifier: Apache-2.0
//
// Independent quality assessment of a proposal pose.  Registration coherence
// alone can be optimistic, so the pose is checked against the raw surface map
// and against the free space encoded in the plan atlas.

#ifndef RELOCALIZER__INSPECTOR_HPP_
#define RELOCALIZER__INSPECTOR_HPP_

#include <vector>

#include <Eigen/Core>

#include "relocalizer/surface_atlas.hpp"
#include "relocalizer/plan_atlas.hpp"
#include "relocalizer/placement.hpp"

namespace relocalizer
{

struct QualitySettings
{
  double surface_radius{0.30};
  double nearest_limit{0.80};
  double open_clearance{0.60};
};

struct Quality
{
  double surface_ratio{0.0};
  double median_distance{0.0};
  double open_space_ratio{0.0};
  double coverage{0.0};
  double pose_clearance{0.0};
  int samples{0};
};

Quality inspect_pose(
  const PlanAtlas & plan, const SurfaceAtlas & atlas,
  const std::vector<Eigen::Vector3d> & points, const Placement & pose,
  const QualitySettings & settings);

}  // namespace relocalizer

#endif  // RELOCALIZER__INSPECTOR_HPP_
