// Copyright 2026 bw_nav contributors
// SPDX-License-Identifier: Apache-2.0

#include "relocalizer/engine.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

namespace relocalizer
{
namespace
{

double clamp01(double value)
{
  return std::max(0.0, std::min(1.0, value));
}

bool passes_gates(const Quality & quality, const EngineSettings & settings)
{
  const double surfaces = quality.surface_ratio * quality.samples;
  return quality.samples >= settings.minimum_surface_points &&
         quality.surface_ratio >= settings.minimum_surface_ratio &&
         surfaces >= settings.minimum_surface_points &&
         quality.open_space_ratio <= settings.maximum_open_ratio &&
         quality.pose_clearance >= settings.minimum_pose_clearance;
}

bool same_place(const Placement & left, const Placement & right)
{
  return std::hypot(left.x - right.x, left.y - right.y) < 1.2 &&
         std::abs(heading_gap(left.heading, right.heading)) < deg_to_rad(40.0);
}

}  // namespace

bool Engine::configure(const EngineSettings & settings, std::string & error)
{
  settings_ = settings;
  if (!plan_.load(settings.plan_descriptor, settings.affinity_sigma, error)) {
    return false;
  }
  if (!atlas_.load(
      settings.surface_map, settings.voxel_spans, settings.voxel_floor,
      settings.voxel_ceiling, settings.bag_span, settings.bag_radius, error))
  {
    return false;
  }
  return true;
}

Outcome Engine::relocalize(const std::vector<Eigen::Vector3d> & surface) const
{
  Outcome outcome;
  const SweepSlice slice = reduce_sweep(surface, settings_.sweep);
  if (!slice.usable()) {
    outcome.explanation = "sweep is too sparse after reduction";
    return outcome;
  }

  const SearchReport report = search_globally(
    plan_, atlas_, slice.search_points, slice.core_points, settings_.align, settings_.search);
  outcome.proposals = report.hypotheses;
  if (report.hypotheses.empty()) {
    outcome.explanation = "no hypothesis survived the global search";
    return outcome;
  }

  struct Judged
  {
    Hypothesis hypothesis;
    Quality quality;
    double rank{0.0};
  };
  std::vector<Judged> judged;
  judged.reserve(report.hypotheses.size());
  for (const Hypothesis & hypothesis : report.hypotheses) {
    Judged ordered;
    ordered.hypothesis = hypothesis;
    ordered.quality = inspect_pose(
      plan_, atlas_, slice.search_points, hypothesis.pose, settings_.quality);
    ordered.rank = ordered.quality.surface_ratio + 0.05 * hypothesis.coherence;
    judged.push_back(ordered);
  }
  std::sort(judged.begin(), judged.end(), [](const Judged & left, const Judged & right) {
    if (left.rank != right.rank) {
      return left.rank > right.rank;
    }
    return left.quality.median_distance < right.quality.median_distance;
  });

  const Judged & leading = judged.front();
  outcome.pose = leading.hypothesis.pose;
  outcome.surface_ratio = leading.quality.surface_ratio;
  outcome.median_distance = leading.quality.median_distance;
  outcome.open_space_ratio = leading.quality.open_space_ratio;
  outcome.plan_agreement = leading.hypothesis.plan_agreement;
  outcome.coherence = leading.hypothesis.coherence;
  outcome.overlap = leading.hypothesis.overlap;

  if (!passes_gates(leading.quality, settings_)) {
    outcome.explanation = "leading hypothesis failed the quality gates";
    return outcome;
  }

  for (std::size_t index = 1; index < judged.size(); ++index) {
    if (same_place(leading.hypothesis.pose, judged[index].hypothesis.pose)) {
      continue;
    }
    if (!passes_gates(judged[index].quality, settings_)) {
      continue;
    }
    if (judged[index].quality.surface_ratio >=
      leading.quality.surface_ratio - settings_.ambiguity_margin)
    {
      outcome.explanation = "several distant places explain the sweep equally well";
      return outcome;
    }
  }

  outcome.confirmed = true;
  outcome.confidence = clamp01(
    leading.quality.surface_ratio * (1.0 - leading.quality.open_space_ratio) *
    (0.55 + 0.45 * leading.hypothesis.coherence));
  outcome.explanation = "confirmed";
  return outcome;
}

AlignReport Engine::refine(
  const std::vector<Eigen::Vector3d> & core_points, const Placement & initial) const
{
  return align_surface(atlas_, core_points, initial, settings_.align);
}

}  // namespace relocalizer
