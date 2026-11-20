// Copyright 2026 bw_nav contributors
// SPDX-License-Identifier: Apache-2.0
//
// Map ownership and the user-facing localization operation.

#ifndef RELOCALIZER__ENGINE_HPP_
#define RELOCALIZER__ENGINE_HPP_

#include <string>
#include <vector>

#include "relocalizer/aligner.hpp"
#include "relocalizer/surface_atlas.hpp"
#include "relocalizer/finder.hpp"
#include "relocalizer/plan_atlas.hpp"
#include "relocalizer/inspector.hpp"
#include "relocalizer/sweep_prep.hpp"

namespace relocalizer
{

struct EngineSettings
{
  std::string plan_descriptor;
  std::string surface_map;
  double affinity_sigma{0.50};
  std::vector<double> voxel_spans{2.0, 1.0};
  double voxel_floor{0.25};
  double voxel_ceiling{3.0};
  double bag_span{0.50};
  double bag_radius{0.80};
  SweepSettings sweep;
  AlignSettings align;
  SearchSettings search;
  QualitySettings quality;
  double minimum_surface_ratio{0.62};
  int minimum_surface_points{60};
  double maximum_open_ratio{0.30};
  double minimum_pose_clearance{0.05};
  double ambiguity_margin{0.08};
};

struct Outcome
{
  bool confirmed{false};
  Placement pose;
  double confidence{0.0};
  double surface_ratio{0.0};
  double median_distance{0.0};
  double open_space_ratio{0.0};
  double plan_agreement{0.0};
  double coherence{0.0};
  double overlap{0.0};
  std::string explanation;
  std::vector<Hypothesis> proposals;
};

class Engine
{
public:
  bool configure(const EngineSettings & settings, std::string & error);
  bool ready() const {return !plan_.empty() && atlas_.tier_count() > 0;}
  const PlanAtlas & plan() const {return plan_;}
  const SurfaceAtlas & atlas() const {return atlas_;}
  const EngineSettings & settings() const {return settings_;}

  /// Full global planar search returning the leading verified pose.
  Outcome relocalize(const std::vector<Eigen::Vector3d> & surface) const;

  /// Improve a pose from a prior estimate; used while tracking.
  AlignReport refine(
    const std::vector<Eigen::Vector3d> & core_points, const Placement & initial) const;

private:
  EngineSettings settings_;
  PlanAtlas plan_;
  SurfaceAtlas atlas_;
};

}  // namespace relocalizer

#endif  // RELOCALIZER__ENGINE_HPP_
