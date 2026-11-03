// Copyright 2026 bw_nav contributors
// SPDX-License-Identifier: Apache-2.0
//
// 2-D plan map ("atlas") loaded from a PGM + map descriptor YAML pair.
//
// Responsibilities:
//   * decode the blocked-cell image into free / blocked / unknown cells,
//   * build a Euclidean distance field to blocked cells,
//   * expose a Gaussian affinity lookup used by the coarse search.
//
// The image row order follows the map-server convention: row 0 of the PGM is
// the maximum-y edge, so the in-memory plan is flipped vertically before use.

#ifndef RELOCALIZER__PLAN_ATLAS_HPP_
#define RELOCALIZER__PLAN_ATLAS_HPP_

#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include <Eigen/Core>

namespace relocalizer
{

class PlanAtlas
{
public:
  struct Descriptor
  {
    std::string raster_path;
    double cell_span{0.05};
    double anchor_x{0.0};
    double anchor_y{0.0};
    double anchor_heading{0.0};
    bool negated{false};
    double blocked_threshold{0.65};
    double free_threshold{0.25};
    std::string mode{"trinary"};
  };

  /// Load a map descriptor (YAML) and its image.  `affinity_sigma` is the
  /// width of the Gaussian centred on blocked cells.
  bool load(const std::string & descriptor_path, double affinity_sigma, std::string & error);

  /// Build a plan from an already decoded blocked mask.  `blocked` uses one
  /// byte per cell, row 0 = minimum y.  Primarily used by tests.
  bool adopt(
    int columns, int rows, double cell_span, double anchor_x, double anchor_y,
    double anchor_heading, std::vector<uint8_t> blocked, double affinity_sigma,
    std::string & error);

  bool empty() const {return columns_ <= 0 || rows_ <= 0;}

  int columns() const {return columns_;}
  int rows() const {return rows_;}
  double cell_span() const {return cell_span_;}
  double anchor_x() const {return anchor_x_;}
  double anchor_y() const {return anchor_y_;}
  double anchor_heading() const {return anchor_heading_;}
  double width() const {return columns_ * cell_span_;}
  double height() const {return rows_ * cell_span_;}
  uint64_t digest() const {return digest_;}
  const std::string & raster_path() const {return raster_path_;}
  const std::vector<uint8_t> & blocked() const {return blocked_;}
  const std::vector<float> & affinity() const {return affinity_;}
  const std::vector<float> & clearance() const {return clearance_;}

  /// Cell index for a world point.  Returns false when outside the plan.
  inline bool locate(double wx, double wy, int & column, int & row) const
  {
    const double dx = wx - anchor_x_;
    const double dy = wy - anchor_y_;
    double gx = dx;
    double gy = dy;
    if (!axis_aligned_) {
      const double c = std::cos(anchor_heading_);
      const double s = std::sin(anchor_heading_);
      gx = c * dx + s * dy;
      gy = -s * dx + c * dy;
    }
    const int col = static_cast<int>(std::floor(gx * inverse_span_));
    const int rw = static_cast<int>(std::floor(gy * inverse_span_));
    if (col < 0 || rw < 0 || col >= columns_ || rw >= rows_) {
      return false;
    }
    column = col;
    row = rw;
    return true;
  }

  inline bool contains(double wx, double wy) const
  {
    int column = 0;
    int row = 0;
    return locate(wx, wy, column, row);
  }

  inline bool blocked_at(double wx, double wy) const
  {
    int column = 0;
    int row = 0;
    return locate(wx, wy, column, row) && blocked_[static_cast<std::size_t>(row) * columns_ + column] != 0;
  }

  /// Nearest-cell affinity in [0, 1]; 0 outside the map.
  inline float affinity_at(double wx, double wy) const
  {
    int column = 0;
    int row = 0;
    if (!locate(wx, wy, column, row)) {
      return 0.0f;
    }
    return affinity_[static_cast<std::size_t>(row) * columns_ + column];
  }

  double clearance_at(double wx, double wy) const;

  /// Lower-left corner of a cell in world coordinates (honours anchor heading).
  Eigen::Vector2d cell_corner(int column, int row) const;

  static bool read_descriptor(const std::string & path, Descriptor & descriptor, std::string & error);
  static bool read_pgm(
    const std::string & path, int & columns, int & rows, int & max_value,
    std::vector<uint16_t> & raster_values, std::string & error);

private:
  bool rebuild_fields(double affinity_sigma, std::string & error);

  int columns_{0};
  int rows_{0};
  double cell_span_{0.05};
  double anchor_x_{0.0};
  double anchor_y_{0.0};
  double anchor_heading_{0.0};
  double affinity_sigma_{0.5};
  double inverse_span_{20.0};
  bool axis_aligned_{true};
  std::vector<uint8_t> blocked_;
  std::vector<float> affinity_;
  std::vector<float> clearance_;
  std::string raster_path_;
  uint64_t digest_{0};
};

}  // namespace relocalizer

#endif  // RELOCALIZER__PLAN_ATLAS_HPP_
