// Copyright 2026 bw_nav contributors
// SPDX-License-Identifier: Apache-2.0

#include "relocalizer/surface_atlas.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstring>
#include <cstdint>
#include <fstream>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "relocalizer/file_digest.hpp"

namespace relocalizer
{
namespace
{

struct PcdLayout
{
  std::vector<std::string> names;
  std::vector<int> sizes;
  std::vector<int> counts;
  std::vector<char> types;
  std::vector<std::size_t> offsets;
  int points{0};
  std::string storage;
  int x_index{-1};
  int y_index{-1};
  int z_index{-1};
  std::size_t record_bytes{0};
};

bool parse_header(std::istream & stream, PcdLayout & layout, std::string & error)
{
  std::string line;
  bool saw_fields = false;
  bool saw_size = false;
  bool saw_type = false;
  bool saw_count = false;
  int width = 0;
  int height = 1;

  while (std::getline(stream, line)) {
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }
    std::istringstream parser(line);
    std::string key;
    parser >> key;
    if (key.empty()) {
      continue;
    }
    std::transform(key.begin(), key.end(), key.begin(), [](unsigned char value) {
      return static_cast<char>(std::toupper(value));
    });

    if (key == "FIELDS") {
      std::string name;
      while (parser >> name) {
        layout.names.push_back(name);
      }
      saw_fields = true;
    } else if (key == "SIZE") {
      int value = 0;
      while (parser >> value) {
        layout.sizes.push_back(value);
      }
      saw_size = true;
    } else if (key == "TYPE") {
      char value = 'F';
      while (parser >> value) {
        layout.types.push_back(value);
      }
      saw_type = true;
    } else if (key == "COUNT") {
      int value = 0;
      while (parser >> value) {
        layout.counts.push_back(value);
      }
      saw_count = true;
    } else if (key == "WIDTH") {
      parser >> width;
    } else if (key == "HEIGHT") {
      parser >> height;
    } else if (key == "POINTS") {
      parser >> layout.points;
    } else if (key == "DATA") {
      parser >> layout.storage;
      break;
    }
  }

  if (!saw_fields || !saw_size) {
    error = "PCD header is missing FIELDS or SIZE";
    return false;
  }
  if (!saw_type) {
    layout.types.assign(layout.names.size(), 'F');
  }
  if (!saw_count) {
    layout.counts.assign(layout.names.size(), 1);
  }
  if (layout.names.size() != layout.sizes.size() ||
    layout.names.size() != layout.types.size() ||
    layout.names.size() != layout.counts.size())
  {
    error = "PCD header field descriptors have inconsistent lengths";
    return false;
  }
  if (layout.points <= 0) {
    layout.points = width * height;
  }
  if (layout.points <= 0) {
    error = "PCD header declares no points";
    return false;
  }

  std::size_t offset = 0;
  for (std::size_t index = 0; index < layout.names.size(); ++index) {
    layout.offsets.push_back(offset);
    offset += static_cast<std::size_t>(layout.sizes[index]) *
      static_cast<std::size_t>(layout.counts[index]);
    if (layout.names[index] == "x") {
      layout.x_index = static_cast<int>(index);
    } else if (layout.names[index] == "y") {
      layout.y_index = static_cast<int>(index);
    } else if (layout.names[index] == "z") {
      layout.z_index = static_cast<int>(index);
    }
  }
  layout.record_bytes = offset;
  if (layout.x_index < 0 || layout.y_index < 0 || layout.z_index < 0) {
    error = "PCD header has no x/y/z fields";
    return false;
  }
  return true;
}

bool read_component(
  const char * block, const PcdLayout & layout, int field, double & value)
{
  const char * location = block + layout.offsets[static_cast<std::size_t>(field)];
  const int bytes = layout.sizes[static_cast<std::size_t>(field)];
  const char kind = layout.types[static_cast<std::size_t>(field)];
  if (kind == 'F') {
    if (bytes == 4) {
      float sample = 0.0F;
      std::memcpy(&sample, location, sizeof(sample));
      value = static_cast<double>(sample);
      return true;
    }
    if (bytes == 8) {
      double sample = 0.0;
      std::memcpy(&sample, location, sizeof(sample));
      value = sample;
      return true;
    }
    return false;
  }
  if (kind == 'I') {
    if (bytes == 1) {
      value = static_cast<double>(*reinterpret_cast<const int8_t *>(location));
    } else if (bytes == 2) {
      int16_t sample = 0;
      std::memcpy(&sample, location, sizeof(sample));
      value = static_cast<double>(sample);
    } else if (bytes == 4) {
      int32_t sample = 0;
      std::memcpy(&sample, location, sizeof(sample));
      value = static_cast<double>(sample);
    } else {
      return false;
    }
    return true;
  }
  if (kind == 'U') {
    if (bytes == 1) {
      value = static_cast<double>(*reinterpret_cast<const uint8_t *>(location));
    } else if (bytes == 2) {
      uint16_t sample = 0;
      std::memcpy(&sample, location, sizeof(sample));
      value = static_cast<double>(sample);
    } else if (bytes == 4) {
      uint32_t sample = 0;
      std::memcpy(&sample, location, sizeof(sample));
      value = static_cast<double>(sample);
    } else {
      return false;
    }
    return true;
  }
  return false;
}

}  // namespace

bool SurfaceAtlas::read_points(
  const std::string & path, std::vector<Eigen::Vector3d> & points, std::string & error)
{
  std::ifstream stream(path, std::ios::binary);
  if (!stream) {
    error = "cannot open surface map: " + path;
    return false;
  }
  PcdLayout layout;
  if (!parse_header(stream, layout, error)) {
    error += ": " + path;
    return false;
  }

  points.clear();
  points.reserve(static_cast<std::size_t>(layout.points));

  if (layout.storage == "binary") {
    const std::vector<char> blob(
      (std::istreambuf_iterator<char>(stream)), std::istreambuf_iterator<char>());
    const std::size_t required =
      static_cast<std::size_t>(layout.points) * layout.record_bytes;
    if (blob.size() < required) {
      error = "surface map payload is truncated: " + path;
      return false;
    }
    for (int index = 0; index < layout.points; ++index) {
      const char * block = blob.data() + static_cast<std::size_t>(index) * layout.record_bytes;
      double sample_x = 0.0;
      double sample_y = 0.0;
      double sample_z = 0.0;
      if (!read_component(block, layout, layout.x_index, sample_x) ||
        !read_component(block, layout, layout.y_index, sample_y) ||
        !read_component(block, layout, layout.z_index, sample_z))
      {
        error = "surface map has an unsupported coordinate representation: " + path;
        return false;
      }
      if (std::isfinite(sample_x) && std::isfinite(sample_y) && std::isfinite(sample_z)) {
        points.emplace_back(sample_x, sample_y, sample_z);
      }
    }
  } else if (layout.storage == "ascii") {
    std::string token;
    for (int index = 0; index < layout.points; ++index) {
      double sample_x = 0.0;
      double sample_y = 0.0;
      double sample_z = 0.0;
      for (std::size_t field = 0; field < layout.names.size(); ++field) {
        for (int repeat = 0; repeat < layout.counts[field]; ++repeat) {
          if (!(stream >> token)) {
            error = "surface map payload is truncated: " + path;
            return false;
          }
          double value = 0.0;
          try {
            value = std::stod(token);
          } catch (const std::exception &) {
            error = "surface map payload contains a non-numeric token: " + path;
            return false;
          }
          if (static_cast<int>(field) == layout.x_index) {
            sample_x = value;
          } else if (static_cast<int>(field) == layout.y_index) {
            sample_y = value;
          } else if (static_cast<int>(field) == layout.z_index) {
            sample_z = value;
          }
        }
      }
      if (std::isfinite(sample_x) && std::isfinite(sample_y) && std::isfinite(sample_z)) {
        points.emplace_back(sample_x, sample_y, sample_z);
      }
    }
  } else {
    error = "unsupported PCD storage mode '" + layout.storage + "': " + path;
    return false;
  }

  if (points.empty()) {
    error = "surface map contains no finite points: " + path;
    return false;
  }
  return true;
}

void PointBag::adopt(
  std::vector<Eigen::Vector3d> samples, double span, double search_radius)
{
  samples_ = std::move(samples);
  span_ = span > 1.0e-6 ? span : 0.5;
  reach_ = std::max(1, static_cast<int>(std::ceil(search_radius / span_)));
  buckets_.clear();
  buckets_.reserve(samples_.size() / 4 + 1);
  for (std::size_t index = 0; index < samples_.size(); ++index) {
    const Eigen::Vector3d & sample = samples_[index];
    CellKey3 key{
      static_cast<int32_t>(std::floor(sample.x() / span_)),
      static_cast<int32_t>(std::floor(sample.y() / span_)),
      static_cast<int32_t>(std::floor(sample.z() / span_))};
    buckets_[key].push_back(static_cast<uint32_t>(index));
  }
}

bool PointBag::nearest(
  const Eigen::Vector3d & query, double radius, Eigen::Vector3d & point,
  double & distance) const
{
  if (samples_.empty()) {
    return false;
  }
  const int reach = std::max(reach_, static_cast<int>(std::ceil(radius / span_)));
  const CellKey3 base{
    static_cast<int32_t>(std::floor(query.x() / span_)),
    static_cast<int32_t>(std::floor(query.y() / span_)),
    static_cast<int32_t>(std::floor(query.z() / span_))};

  double leading = radius * radius;
  bool found = false;
  Eigen::Vector3d leading_point = Eigen::Vector3d::Zero();
  for (int offset_x = -reach; offset_x <= reach; ++offset_x) {
    for (int offset_y = -reach; offset_y <= reach; ++offset_y) {
      for (int offset_z = -reach; offset_z <= reach; ++offset_z) {
        const CellKey3 key{base.i + offset_x, base.j + offset_y, base.k + offset_z};
        const auto bucket = buckets_.find(key);
        if (bucket == buckets_.end()) {
          continue;
        }
        for (uint32_t index : bucket->second) {
          const double squared = (samples_[index] - query).squaredNorm();
          if (squared <= leading) {
            leading = squared;
            leading_point = samples_[index];
            found = true;
          }
        }
      }
    }
  }
  if (!found) {
    return false;
  }
  point = leading_point;
  distance = std::sqrt(leading);
  return true;
}

bool VoxelTier::find(const Eigen::Vector3d & query, uint32_t & index) const
{
  const CellKey3 key{
    static_cast<int32_t>(std::floor(query.x() / span)),
    static_cast<int32_t>(std::floor(query.y() / span)),
    static_cast<int32_t>(std::floor(query.z() / span))};
  const auto found = lookup.find(key);
  if (found == lookup.end()) {
    return false;
  }
  index = found->second;
  return true;
}

bool SurfaceAtlas::load(
  const std::string & path, const std::vector<double> & spans, double lowest,
  double highest, double bag_span, double bag_radius, std::string & error)
{
  std::vector<Eigen::Vector3d> points;
  if (!read_points(path, points, error)) {
    return false;
  }
  digest_ = digest_of_file(path);
  samples_ = points;
  bag_.adopt(points, bag_span, bag_radius);

  tiers_.clear();
  for (double span : spans) {
    if (!(span > 0.05)) {
      error = "voxel tier resolution is too small";
      return false;
    }
    VoxelTier tier;
    tier.span = span;
    struct Accumulator
    {
      Eigen::Vector3d sum{Eigen::Vector3d::Zero()};
      Eigen::Matrix3d outer{Eigen::Matrix3d::Zero()};
      int count{0};
    };
    std::vector<Accumulator> accumulators;
    accumulators.reserve(samples_.size() / 16 + 1);
    tier.lookup.reserve(samples_.size() / 16 + 1);

    for (const Eigen::Vector3d & sample : samples_) {
      if (sample.z() < lowest || sample.z() > highest) {
        continue;
      }
      const CellKey3 key{
        static_cast<int32_t>(std::floor(sample.x() / span)),
        static_cast<int32_t>(std::floor(sample.y() / span)),
        static_cast<int32_t>(std::floor(sample.z() / span))};
      const auto found = tier.lookup.find(key);
      uint32_t index = 0;
      if (found == tier.lookup.end()) {
        index = static_cast<uint32_t>(accumulators.size());
        accumulators.emplace_back();
        tier.lookup.emplace(key, index);
      } else {
        index = found->second;
      }
      Accumulator & accumulator = accumulators[index];
      accumulator.sum += sample;
      accumulator.outer += sample * sample.transpose();
      ++accumulator.count;
    }

    tier.means.reserve(accumulators.size());
    tier.covariances.reserve(accumulators.size());
    tier.populations.reserve(accumulators.size());
    for (const Accumulator & accumulator : accumulators) {
      if (accumulator.count < 2) {
        continue;
      }
      const Eigen::Vector3d mean = accumulator.sum / static_cast<double>(accumulator.count);
      Eigen::Matrix3d covariance =
        accumulator.outer / static_cast<double>(accumulator.count) - mean * mean.transpose();
      covariance = 0.5 * (covariance + covariance.transpose());
      covariance.diagonal().array() += 1.0e-2;
      tier.means.push_back(mean);
      tier.covariances.push_back(covariance);
      tier.populations.push_back(accumulator.count);
    }
    // Rebuild the lookup because voxels with a single sample were dropped.
    tier.lookup.clear();
    tier.lookup.reserve(tier.means.size());
    for (std::size_t index = 0; index < tier.means.size(); ++index) {
      const Eigen::Vector3d & mean = tier.means[index];
      const CellKey3 key{
        static_cast<int32_t>(std::floor(mean.x() / span)),
        static_cast<int32_t>(std::floor(mean.y() / span)),
        static_cast<int32_t>(std::floor(mean.z() / span))};
      tier.lookup.emplace(key, static_cast<uint32_t>(index));
    }
    tiers_.push_back(std::move(tier));
  }
  return true;
}

}  // namespace relocalizer
