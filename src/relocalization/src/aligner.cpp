// Copyright 2026 bw_nav contributors
// SPDX-License-Identifier: Apache-2.0

#include "relocalizer/aligner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include <Eigen/Cholesky>
#include <Eigen/LU>

namespace relocalizer
{
namespace
{

struct TierOrder
{
  int index;
  double span;
};

}  // namespace

AlignReport align_surface(
  const SurfaceAtlas & atlas, const std::vector<Eigen::Vector3d> & sweep,
  const Placement & initial, const AlignSettings & settings)
{
  AlignReport report;
  report.pose = initial;
  if (atlas.tier_count() == 0 || sweep.empty()) {
    return report;
  }

  std::vector<TierOrder> order;
  order.reserve(static_cast<std::size_t>(atlas.tier_count()));
  for (int index = 0; index < atlas.tier_count(); ++index) {
    order.push_back(TierOrder{index, atlas.tier(index).span});
  }
  std::sort(order.begin(), order.end(), [](const TierOrder & left, const TierOrder & right) {
    return left.span < right.span;
  });

  Placement pose = initial;
  for (int iteration = 0; iteration < settings.max_iterations; ++iteration) {
    Eigen::Matrix3d normal = Eigen::Matrix3d::Zero();
    Eigen::Vector3d gradient = Eigen::Vector3d::Zero();
    double penalty = 0.0;
    int used = 0;
    const Eigen::Matrix2d rotation = heading_matrix(pose.heading);

    for (const Eigen::Vector3d & point : sweep) {
      const Eigen::Vector2d local(point.x(), point.y());
      const Eigen::Vector2d rotated = rotation * local;
      const Eigen::Vector3d world(rotated.x() + pose.x, rotated.y() + pose.y, point.z());

      const VoxelTier * hit_tier = nullptr;
      uint32_t voxel = 0;
      for (const TierOrder & proposal : order) {
        const VoxelTier & tier = atlas.tier(proposal.index);
        if (tier.find(world, voxel)) {
          hit_tier = &tier;
          break;
        }
      }
      if (hit_tier == nullptr) {
        continue;
      }

      const Eigen::Vector3d delta = world - hit_tier->means[voxel];
      const Eigen::Matrix3d information = hit_tier->covariances[voxel].inverse();

      const Eigen::Vector3d pull = information * delta;
      // Huber-style down-weighting keeps floor and ceiling samples from
      // dominating a mostly wall-shaped map.
      const double squared = delta.dot(pull);
      const double weight = squared > 9.0 ? 3.0 / std::sqrt(squared) : 1.0;
      // Derivative of the world point with respect to (x, y, heading).  The
      // z row is zero because the pose has no vertical component; the damping
      // term below keeps the resulting normal equations solvable.
      Eigen::Matrix3d jacobian;
      jacobian << 1.0, 0.0, -rotated.y(),
        0.0, 1.0, rotated.x(),
        0.0, 0.0, 0.0;
      normal.noalias() += weight * jacobian.transpose() * information * jacobian;
      gradient.noalias() -= weight * jacobian.transpose() * pull;
      penalty += squared;
      ++used;
    }

    if (used < settings.minimum_overlap) {
      break;
    }
    normal.diagonal().array() += settings.damping;
    const Eigen::Vector3d step = normal.ldlt().solve(gradient);
    if (!step.allFinite()) {
      break;
    }
    pose.x += step.x();
    pose.y += step.y();
    pose.heading = wrap_heading(pose.heading + step.z());
    report.iterations = iteration + 1;

    if (std::hypot(step.x(), step.y()) < settings.step_tolerance &&
      std::abs(step.z()) < settings.step_tolerance)
    {
      report.converged = true;
      break;
    }
    if (std::hypot(step.x(), step.y()) > settings.maximum_shift ||
      std::abs(step.z()) > settings.maximum_turn)
    {
      break;
    }
    if (std::abs(pose.x - initial.x) > 1.5 * settings.maximum_shift ||
      std::abs(pose.y - initial.y) > 1.5 * settings.maximum_shift)
    {
      break;
    }
  }

  // One final scoring pass with the converged pose so the reported coherence is
  // independent of the last confirmed step.
  const Eigen::Matrix2d rotation = heading_matrix(pose.heading);
  double penalty = 0.0;
  int used = 0;
  int inliers = 0;
  for (const Eigen::Vector3d & point : sweep) {
    const Eigen::Vector3d world(
      rotation(0, 0) * point.x() + rotation(0, 1) * point.y() + pose.x,
      rotation(1, 0) * point.x() + rotation(1, 1) * point.y() + pose.y,
      point.z());
    const VoxelTier * hit_tier = nullptr;
    uint32_t voxel = 0;
    for (const TierOrder & proposal : order) {
      const VoxelTier & tier = atlas.tier(proposal.index);
      if (tier.find(world, voxel)) {
        hit_tier = &tier;
        break;
      }
    }
    if (hit_tier == nullptr) {
      continue;
    }
    const Eigen::Vector3d delta = world - hit_tier->means[voxel];
    const double squared = delta.dot(hit_tier->covariances[voxel].inverse() * delta);
    penalty += squared;
    if (squared <= 9.0) {
      ++inliers;
    }
    ++used;
  }
  report.pose = pose;
  report.used = used;
  report.overlap = sweep.empty() ? 0.0 : static_cast<double>(used) / sweep.size();
  report.mean_penalty = used == 0 ? 0.0 : penalty / used;
  report.coherence = used == 0 ? 0.0 : static_cast<double>(inliers) / used;
  return report;
}

}  // namespace relocalizer
