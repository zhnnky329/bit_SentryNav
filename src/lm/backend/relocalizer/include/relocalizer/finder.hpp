// Copyright 2026 bw_nav contributors
// SPDX-License-Identifier: Apache-2.0
//
// Global hypothesis search.  A coarse exhaustive sweep over the site is
// followed by a local sweep around every promising cell; the survivors are
// improved with the Gaussian registration and returned for verification.

#ifndef RELOCALIZER__FINDER_HPP_
#define RELOCALIZER__FINDER_HPP_

#include <vector>

#include "relocalizer/aligner.hpp"
#include "relocalizer/surface_atlas.hpp"
#include "relocalizer/plan_atlas.hpp"
#include "relocalizer/placement.hpp"

namespace relocalizer
{

struct SearchSettings
{
  double heading_step_deg{10.0};
  double coarse_step{0.40};
  double coarse_floor{0.45};
  int max_positions{22000};
  int coarse_keep{200};
  double fine_step{0.20};
  double fine_heading_deg{2.0};
  double fine_floor{0.55};
  int fine_keep{24};
  double place_separation{1.0};
  double heading_separation_deg{25.0};
  int keep_hypotheses{12};
  int threads{0};
};

struct Hypothesis
{
  Placement pose;
  double plan_agreement{0.0};
  double coherence{0.0};
  double overlap{0.0};
  double mean_penalty{0.0};
  int used{0};
};

struct SearchReport
{
  std::vector<Hypothesis> hypotheses;
  int positions{0};
  int coarse_sparks{0};
  int fine_proposals{0};
};

SearchReport search_globally(
  const PlanAtlas & plan, const SurfaceAtlas & atlas,
  const std::vector<Eigen::Vector3d> & search_points,
  const std::vector<Eigen::Vector3d> & core_points,
  const AlignSettings & align_settings, const SearchSettings & settings);

}  // namespace relocalizer

#endif  // RELOCALIZER__FINDER_HPP_
