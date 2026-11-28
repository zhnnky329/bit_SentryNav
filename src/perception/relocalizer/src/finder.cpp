// Copyright 2026 bw_nav contributors
// SPDX-License-Identifier: Apache-2.0

#include "relocalizer/finder.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <utility>
#include <vector>

#ifdef _OPENMP
#include <omp.h>
#endif

namespace relocalizer
{
namespace
{

struct Spark
{
  double agreement{0.0};
  double x{0.0};
  double y{0.0};
  double heading{0.0};
};

double agreement_at(
  const PlanAtlas & plan, const std::vector<Eigen::Vector2f> & points,
  double cosine, double sine, double tx, double ty)
{
  double total = 0.0;
  for (const Eigen::Vector2f & point : points) {
    const double wx = cosine * point.x() - sine * point.y() + tx;
    const double wy = sine * point.x() + cosine * point.y() + ty;
    total += plan.affinity_at(wx, wy);
  }
  return total / static_cast<double>(points.size());
}

bool too_close(
  const std::vector<Spark> & kept, const Spark & proposal, double place_separation,
  double heading_separation)
{
  for (const Spark & other : kept) {
    const double distance = std::hypot(proposal.x - other.x, proposal.y - other.y);
    const double turn = std::abs(heading_gap(proposal.heading, other.heading));
    if (distance < place_separation && turn < heading_separation) {
      return true;
    }
  }
  return false;
}

}  // namespace

SearchReport search_globally(
  const PlanAtlas & plan, const SurfaceAtlas & atlas,
  const std::vector<Eigen::Vector3d> & search_points,
  const std::vector<Eigen::Vector3d> & core_points,
  const AlignSettings & align_settings, const SearchSettings & settings)
{
  SearchReport report;
  if (search_points.size() < 40 || core_points.empty() || plan.empty() ||
    atlas.tier_count() == 0)
  {
    return report;
  }

  double min_x = std::numeric_limits<double>::max();
  double max_x = std::numeric_limits<double>::lowest();
  double min_y = std::numeric_limits<double>::max();
  double max_y = std::numeric_limits<double>::lowest();
  for (const Eigen::Vector3d & sample : atlas.samples()) {
    min_x = std::min(min_x, sample.x());
    max_x = std::max(max_x, sample.x());
    min_y = std::min(min_y, sample.y());
    max_y = std::max(max_y, sample.y());
  }
  const double span_x = std::max(2.0, max_x - min_x);
  const double span_y = std::max(2.0, max_y - min_y);
  const double area = span_x * span_y;

  double step = settings.coarse_step;
  if (area / (step * step) > settings.max_positions) {
    step = std::sqrt(area / settings.max_positions);
  }
  step = std::max(step, 0.15);
  const int columns = static_cast<int>(std::floor(span_x / step)) + 1;
  const int rows = static_cast<int>(std::floor(span_y / step)) + 1;
  const int turn_count = std::max(1, static_cast<int>(std::round(360.0 / settings.heading_step_deg)));
  report.positions = columns * rows * turn_count;

  std::vector<Eigen::Vector2f> sweep_2d;
  sweep_2d.reserve(search_points.size());
  for (const Eigen::Vector3d & point : search_points) {
    sweep_2d.emplace_back(static_cast<float>(point.x()), static_cast<float>(point.y()));
  }

  std::vector<std::vector<Eigen::Vector2f>> turned(static_cast<std::size_t>(turn_count));
  for (int turn = 0; turn < turn_count; ++turn) {
    const double heading = -kPi + turn * (kTurn / turn_count);
    const double cosine = std::cos(heading);
    const double sine = std::sin(heading);
    turned[turn].reserve(sweep_2d.size());
    for (const Eigen::Vector2f & point : sweep_2d) {
      turned[turn].emplace_back(
        static_cast<float>(cosine * point.x() - sine * point.y()),
        static_cast<float>(sine * point.x() + cosine * point.y()));
    }
  }

  int worker_count = 1;
#ifdef _OPENMP
  if (settings.threads > 0) {
    omp_set_num_threads(settings.threads);
  }
  worker_count = omp_get_max_threads();
#else
  (void)settings.threads;
#endif
  std::vector<std::vector<Spark>> buckets(static_cast<std::size_t>(worker_count));

  const double coarse_gate = settings.coarse_floor;

#ifdef _OPENMP
#pragma omp parallel
#endif
  {
    int worker = 0;
#ifdef _OPENMP
    worker = omp_get_thread_num();
#endif
    std::vector<Spark> & local = buckets[static_cast<std::size_t>(worker)];
#ifdef _OPENMP
#pragma omp for schedule(dynamic, 1)
#endif
    for (int task = 0; task < turn_count * columns; ++task) {
      const int turn = task / columns;
      const int column = task % columns;
      const double heading = -kPi + turn * (kTurn / turn_count);
      const std::vector<Eigen::Vector2f> & points = turned[static_cast<std::size_t>(turn)];
      const double base_x = min_x + column * step;
      for (int row = 0; row < rows; ++row) {
        const double base_y = min_y + row * step;
        const double support = agreement_at(
          plan, points, std::cos(heading), std::sin(heading), base_x, base_y);
        if (support >= coarse_gate) {
          local.push_back(Spark{support, base_x, base_y, heading});
        }
      }
    }
  }

  std::vector<Spark> sparks;
  for (const std::vector<Spark> & bucket : buckets) {
    sparks.insert(sparks.end(), bucket.begin(), bucket.end());
  }
  std::sort(sparks.begin(), sparks.end(), [](const Spark & left, const Spark & right) {
    return left.agreement > right.agreement;
  });

  const double heading_separation = deg_to_rad(settings.heading_separation_deg);
  std::vector<Spark> kept;
  kept.reserve(static_cast<std::size_t>(settings.coarse_keep));
  for (const Spark & spark : sparks) {
    if (too_close(kept, spark, settings.place_separation, heading_separation)) {
      continue;
    }
    kept.push_back(spark);
    if (static_cast<int>(kept.size()) >= settings.coarse_keep) {
      break;
    }
  }
  report.coarse_sparks = static_cast<int>(kept.size());

  std::vector<Spark> fine;
  const int fine_offsets[5] = {-2, -1, 0, 1, 2};
  for (const Spark & spark : kept) {
    for (int offset_x : fine_offsets) {
      for (int offset_y : fine_offsets) {
        const double tx = spark.x + offset_x * settings.fine_step;
        const double ty = spark.y + offset_y * settings.fine_step;
        for (int offset_h : fine_offsets) {
          const double heading =
            spark.heading + offset_h * deg_to_rad(settings.fine_heading_deg);
          const double cosine = std::cos(heading);
          const double sine = std::sin(heading);
          const double support = agreement_at(plan, sweep_2d, cosine, sine, tx, ty);
          if (support >= settings.fine_floor) {
            fine.push_back(Spark{support, tx, ty, heading});
          }
        }
      }
    }
  }
  report.fine_proposals = static_cast<int>(fine.size());
  std::sort(fine.begin(), fine.end(), [](const Spark & left, const Spark & right) {
    return left.agreement > right.agreement;
  });

  std::vector<Spark> finalists;
  finalists.reserve(static_cast<std::size_t>(settings.fine_keep));
  for (const Spark & spark : fine) {
    if (too_close(finalists, spark, 0.6, deg_to_rad(15.0))) {
      continue;
    }
    finalists.push_back(spark);
    if (static_cast<int>(finalists.size()) >= settings.fine_keep) {
      break;
    }
  }

  std::vector<Hypothesis> hypotheses;
  hypotheses.reserve(finalists.size());
  for (const Spark & spark : finalists) {
    Placement initial;
    initial.x = spark.x;
    initial.y = spark.y;
    initial.heading = wrap_heading(spark.heading);
    const AlignReport aligned = align_surface(atlas, core_points, initial, align_settings);
    Hypothesis hypothesis;
    hypothesis.pose = aligned.pose;
    hypothesis.plan_agreement = spark.agreement;
    hypothesis.coherence = aligned.coherence;
    hypothesis.overlap = aligned.overlap;
    hypothesis.used = aligned.used;
    hypothesis.mean_penalty = aligned.mean_penalty;
    hypotheses.push_back(hypothesis);
  }

  std::sort(hypotheses.begin(), hypotheses.end(), [](const Hypothesis & left, const Hypothesis & right) {
    if (left.coherence != right.coherence) {
      return left.coherence > right.coherence;
    }
    return left.plan_agreement > right.plan_agreement;
  });

  std::vector<Hypothesis> distinct;
  distinct.reserve(hypotheses.size());
  for (const Hypothesis & hypothesis : hypotheses) {
    bool duplicate = false;
    for (const Hypothesis & confirmed : distinct) {
      const double distance =
        std::hypot(hypothesis.pose.x - confirmed.pose.x, hypothesis.pose.y - confirmed.pose.y);
      const double turn =
        std::abs(heading_gap(hypothesis.pose.heading, confirmed.pose.heading));
      if (distance < 0.6 && turn < deg_to_rad(15.0)) {
        duplicate = true;
        break;
      }
    }
    if (!duplicate) {
      distinct.push_back(hypothesis);
    }
  }
  if (static_cast<int>(distinct.size()) > settings.keep_hypotheses) {
    distinct.resize(static_cast<std::size_t>(settings.keep_hypotheses));
  }
  report.hypotheses = std::move(distinct);
  return report;
}

}  // namespace relocalizer
