// Copyright 2026 bw_nav contributors
// SPDX-License-Identifier: Apache-2.0
//
// Three-degree-of-freedom registration against the surface atlas.  Every sweep
// point contributes a Gaussian pull from the voxel that contains it; the
// normal equations are solved with a small Levenberg-Marquardt damping term.

#ifndef RELOCALIZER__ALIGNER_HPP_
#define RELOCALIZER__ALIGNER_HPP_

#include <vector>

#include <Eigen/Core>

#include "relocalizer/surface_atlas.hpp"
#include "relocalizer/placement.hpp"

namespace relocalizer
{

struct AlignSettings
{
  int max_iterations{40};
  double step_tolerance{1.0e-4};
  double maximum_shift{0.8};
  double maximum_turn{0.5};
  double damping{1.0e-4};
  int minimum_overlap{25};
};

struct AlignReport
{
  Placement pose;
  double coherence{0.0};
  double overlap{0.0};
  int used{0};
  int iterations{0};
  double mean_penalty{0.0};
  bool converged{false};
};

AlignReport align_surface(
  const SurfaceAtlas & atlas, const std::vector<Eigen::Vector3d> & sweep,
  const Placement & initial, const AlignSettings & settings);

}  // namespace relocalizer

#endif  // RELOCALIZER__ALIGNER_HPP_
