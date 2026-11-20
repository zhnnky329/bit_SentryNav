// Copyright 2026 bw_nav contributors
// SPDX-License-Identifier: Apache-2.0
//
// Surface map access.  A PCD file is decoded once and made available in two
// forms:
//   * a plain point bag with a uniform hash plan for nearest-surface checks,
//   * one or more voxel tiers holding Gaussian statistics for registration.

#ifndef RELOCALIZER__SURFACE_ATLAS_HPP_
#define RELOCALIZER__SURFACE_ATLAS_HPP_

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>

#include "relocalizer/voxel_key.hpp"

namespace relocalizer
{

/// Uniform spatial hash over 3-D points, used for nearest-sample queries.
class PointBag
{
public:
  bool load(const std::string & path, std::string & error);
  void adopt(std::vector<Eigen::Vector3d> samples, double span, double search_radius);

  std::size_t size() const {return samples_.size();}
  const std::vector<Eigen::Vector3d> & samples() const {return samples_;}

  /// Nearest sample within `radius` of the query.  Returns false when the
  /// neighbourhood is empty.
  bool nearest(
    const Eigen::Vector3d & query, double radius, Eigen::Vector3d & point,
    double & distance) const;

private:
  std::vector<Eigen::Vector3d> samples_;
  std::unordered_map<CellKey3, std::vector<uint32_t>, CellKey3Hash> buckets_;
  double span_{0.5};
  int reach_{1};
};

/// One resolution of the Gaussian mixture built from the surface map.
struct VoxelTier
{
  double span{1.0};
  std::unordered_map<CellKey3, uint32_t, CellKey3Hash> lookup;
  std::vector<Eigen::Vector3d> means;
  std::vector<Eigen::Matrix3d> covariances;
  std::vector<int> populations;

  /// Look up the voxel containing a world point.
  bool find(const Eigen::Vector3d & query, uint32_t & index) const;
};

class SurfaceAtlas
{
public:
  /// Decode a PCD file and build the voxel tiers.  Voxel statistics are built
  /// from samples whose height lies in [lowest, highest]; the point bag always
  /// keeps every sample.
  bool load(
    const std::string & path, const std::vector<double> & spans, double lowest,
    double highest, double bag_span, double bag_radius, std::string & error);

  bool empty() const {return samples_.size() == 0;}
  std::size_t point_count() const {return samples_.size();}
  const std::vector<Eigen::Vector3d> & samples() const {return samples_;}
  const PointBag & bag() const {return bag_;}
  int tier_count() const {return static_cast<int>(tiers_.size());}
  const VoxelTier & tier(int index) const {return tiers_.at(static_cast<std::size_t>(index));}
  uint64_t digest() const {return digest_;}

  /// Decode a PCD file into points without building any index.
  static bool read_points(
    const std::string & path, std::vector<Eigen::Vector3d> & points, std::string & error);

private:
  std::vector<Eigen::Vector3d> samples_;
  PointBag bag_;
  std::vector<VoxelTier> tiers_;
  uint64_t digest_{0};
};

}  // namespace relocalizer

#endif  // RELOCALIZER__SURFACE_ATLAS_HPP_
