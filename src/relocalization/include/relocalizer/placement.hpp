// Copyright 2026 bw_nav contributors
// SPDX-License-Identifier: Apache-2.0
//
// Small planar transform helpers shared by the relocalizer.  The whole package
// deliberately works with a 2-D robot pose (x, y, heading); height is a fixed
// property of the site and is supplied as a configuration value.

#ifndef RELOCALIZER__PLACEMENT_HPP_
#define RELOCALIZER__PLACEMENT_HPP_

#include <cmath>

#include <Eigen/Core>

namespace relocalizer
{

constexpr double kPi = 3.14159265358979323846264338327950288;
constexpr double kTurn = 2.0 * kPi;

inline double wrap_heading(double angle)
{
  double folded = std::fmod(angle + kPi, kTurn);
  if (folded < 0.0) {
    folded += kTurn;
  }
  return folded - kPi;
}

inline double deg_to_rad(double value)
{
  return value * kPi / 180.0;
}

inline double rad_to_deg(double value)
{
  return value * 180.0 / kPi;
}

/// Rigid transform on the plane: world_point = R(heading) * body_point + (x, y).
struct Placement
{
  double x{0.0};
  double y{0.0};
  double heading{0.0};

  Eigen::Vector2d translation() const {return Eigen::Vector2d(x, y);}
};

inline Eigen::Matrix2d heading_matrix(double heading)
{
  const double c = std::cos(heading);
  const double s = std::sin(heading);
  Eigen::Matrix2d matrix;
  matrix << c, -s,
    s, c;
  return matrix;
}

inline Eigen::Vector2d rotate_point(const Eigen::Vector2d & point, double heading)
{
  const double c = std::cos(heading);
  const double s = std::sin(heading);
  return Eigen::Vector2d(c * point.x() - s * point.y(), s * point.x() + c * point.y());
}

/// Smallest signed angle that takes `to` onto `from`.
inline double heading_gap(double from, double to)
{
  return wrap_heading(from - to);
}

inline Placement compose(const Placement & outer, const Placement & inner)
{
  const Eigen::Vector2d offset = rotate_point(Eigen::Vector2d(inner.x, inner.y), outer.heading);
  Placement result;
  result.x = outer.x + offset.x();
  result.y = outer.y + offset.y();
  result.heading = wrap_heading(outer.heading + inner.heading);
  return result;
}

inline Placement invert(const Placement & pose)
{
  const Eigen::Vector2d back = rotate_point(Eigen::Vector2d(-pose.x, -pose.y), -pose.heading);
  Placement result;
  result.x = back.x();
  result.y = back.y();
  result.heading = wrap_heading(-pose.heading);
  return result;
}

}  // namespace relocalizer

#endif  // RELOCALIZER__PLACEMENT_HPP_
