// Copyright 2026 bw_nav contributors
// SPDX-License-Identifier: Apache-2.0

#include "relocalizer/inspector.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace relocalizer
{

Quality inspect_pose(
  const PlanAtlas & plan, const SurfaceAtlas & atlas,
  const std::vector<Eigen::Vector3d> & points, const Placement & pose,
  const QualitySettings & settings)
{
  Quality quality;
  quality.samples = static_cast<int>(points.size());
  if (points.empty()) {
    return quality;
  }

  const Eigen::Matrix2d rotation = heading_matrix(pose.heading);
  const PointBag & bag = atlas.bag();
  std::vector<double> distances;
  distances.reserve(points.size());
  int surfaces = 0;
  int covered = 0;
  int mapped = 0;
  int open = 0;

  for (const Eigen::Vector3d & point : points) {
    const Eigen::Vector2d local(point.x(), point.y());
    const Eigen::Vector2d rotated = rotation * local;
    const double wx = rotated.x() + pose.x;
    const double wy = rotated.y() + pose.y;
    const Eigen::Vector3d world(wx, wy, point.z());

    Eigen::Vector3d nearest;
    double distance = settings.nearest_limit;
    if (bag.nearest(world, settings.nearest_limit, nearest, distance)) {
      ++covered;
    }
    distances.push_back(distance);
    if (distance <= settings.surface_radius) {
      ++surfaces;
    }

    if (plan.contains(wx, wy)) {
      ++mapped;
      if (plan.clearance_at(wx, wy) > settings.open_clearance) {
        ++open;
      }
    }
  }

  const double count = static_cast<double>(points.size());
  quality.surface_ratio = surfaces / count;
  quality.coverage = covered / count;
  quality.open_space_ratio = mapped == 0 ? 0.0 : static_cast<double>(open) / mapped;
  quality.pose_clearance = plan.clearance_at(pose.x, pose.y);
  std::sort(distances.begin(), distances.end());
  quality.median_distance = distances[distances.size() / 2];
  return quality;
}

}  // namespace relocalizer
