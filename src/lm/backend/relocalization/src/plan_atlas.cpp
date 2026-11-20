// Copyright 2026 bw_nav contributors
// SPDX-License-Identifier: Apache-2.0

#include "relocalizer/plan_atlas.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "relocalizer/file_digest.hpp"
#include "relocalizer/placement.hpp"

namespace relocalizer
{
namespace
{

constexpr float kFarAway = 1.0e20f;

void transform_line(
  const float * input, int count, float * destination,
  std::vector<int> & sites, std::vector<float> & boundaries)
{
  int last = 0;
  sites[0] = 0;
  boundaries[0] = -std::numeric_limits<float>::infinity();
  boundaries[1] = std::numeric_limits<float>::infinity();
  for (int position = 1; position < count; ++position) {
    float crossing = ((input[position] + position * position) -
      (input[sites[last]] + sites[last] * sites[last])) /
      static_cast<float>(2 * position - 2 * sites[last]);
    while (crossing <= boundaries[last]) {
      --last;
      crossing = ((input[position] + position * position) -
        (input[sites[last]] + sites[last] * sites[last])) /
        static_cast<float>(2 * position - 2 * sites[last]);
    }
    ++last;
    sites[last] = position;
    boundaries[last] = crossing;
    boundaries[last + 1] = std::numeric_limits<float>::infinity();
  }
  last = 0;
  for (int position = 0; position < count; ++position) {
    while (boundaries[last + 1] < position) {
      ++last;
    }
    const float delta = static_cast<float>(position - sites[last]);
    destination[position] = delta * delta + input[sites[last]];
  }
}

/// Squared Euclidean distance (in cells) from every cell to the nearest
/// blocked cell, using the exact Felzenszwalb-Huttenlocher transform.
void squared_distance_field(
  const std::vector<uint8_t> & blocked, int columns, int rows,
  std::vector<float> & squared)
{
  squared.assign(static_cast<std::size_t>(columns) * rows, kFarAway);
  for (std::size_t index = 0; index < blocked.size(); ++index) {
    if (blocked[index] != 0) {
      squared[index] = 0.0f;
    }
  }

  const int span = std::max(columns, rows);
  std::vector<float> input(span);
  std::vector<float> destination(span);
  std::vector<int> sites(span);
  std::vector<float> boundaries(span + 1);

  for (int row = 0; row < rows; ++row) {
    float * line = squared.data() + static_cast<std::size_t>(row) * columns;
    std::copy(line, line + columns, input.begin());
    transform_line(input.data(), columns, line, sites, boundaries);
  }
  for (int column = 0; column < columns; ++column) {
    for (int row = 0; row < rows; ++row) {
      input[row] = squared[static_cast<std::size_t>(row) * columns + column];
    }
    transform_line(input.data(), rows, destination.data(), sites, boundaries);
    for (int row = 0; row < rows; ++row) {
      squared[static_cast<std::size_t>(row) * columns + column] = destination[row];
    }
  }
}

bool read_numeric_token(
  const std::string & bytes, std::size_t & cursor, std::string & token)
{
  while (cursor < bytes.size()) {
    const unsigned char symbol = static_cast<unsigned char>(bytes[cursor]);
    if (symbol == '#') {
      while (cursor < bytes.size() && bytes[cursor] != '\n') {
        ++cursor;
      }
      continue;
    }
    if (std::isspace(symbol) != 0) {
      ++cursor;
      continue;
    }
    break;
  }
  if (cursor >= bytes.size()) {
    return false;
  }
  const std::size_t begin = cursor;
  while (cursor < bytes.size()) {
    const unsigned char symbol = static_cast<unsigned char>(bytes[cursor]);
    if (std::isspace(symbol) != 0 || symbol == '#') {
      break;
    }
    ++cursor;
  }
  token.assign(bytes, begin, cursor - begin);
  return true;
}

bool parse_positive(const std::string & token, long & value)
{
  try {
    std::size_t used = 0;
    const long parsed = std::stol(token, &used);
    if (used != token.size() || parsed <= 0) {
      return false;
    }
    value = parsed;
    return true;
  } catch (const std::exception &) {
    return false;
  }
}

}  // namespace

bool PlanAtlas::read_descriptor(
  const std::string & path, Descriptor & descriptor, std::string & error)
{
  try {
    const YAML::Node root = YAML::LoadFile(path);
    if (!root.IsMap()) {
      error = "map descriptor is not a mapping: " + path;
      return false;
    }
    if (!root["image"]) {
      error = "map descriptor has no 'image' entry: " + path;
      return false;
    }
    descriptor.raster_path = root["image"].as<std::string>();
    if (descriptor.raster_path.empty()) {
      error = "map descriptor has an empty 'image' entry: " + path;
      return false;
    }
    if (descriptor.raster_path.front() != '/') {
      const std::string folder = path.substr(0, path.find_last_of('/') + 1);
      descriptor.raster_path = folder + descriptor.raster_path;
    }
    descriptor.cell_span = root["resolution"] ? root["resolution"].as<double>() : 0.05;
    if (!(descriptor.cell_span > 0.0)) {
      error = "map resolution must be positive: " + path;
      return false;
    }
    if (root["origin"] && root["origin"].IsSequence() && root["origin"].size() >= 2) {
      descriptor.anchor_x = root["origin"][0].as<double>();
      descriptor.anchor_y = root["origin"][1].as<double>();
      if (root["origin"].size() >= 3) {
        descriptor.anchor_heading = root["origin"][2].as<double>();
      }
    }
    descriptor.negated = root["negate"] && root["negate"].as<int>() != 0;
    descriptor.blocked_threshold =
      root["occupied_thresh"] ? root["occupied_thresh"].as<double>() : 0.65;
    descriptor.free_threshold =
      root["free_thresh"] ? root["free_thresh"].as<double>() : 0.25;
    if (root["mode"]) {
      descriptor.mode = root["mode"].as<std::string>();
    }
  } catch (const YAML::Exception & exception) {
    error = "cannot parse map descriptor " + path + ": " + exception.what();
    return false;
  }
  return true;
}

bool PlanAtlas::read_pgm(
  const std::string & path, int & columns, int & rows, int & max_value,
  std::vector<uint16_t> & raster_values, std::string & error)
{
  std::ifstream stream(path, std::ios::binary);
  if (!stream) {
    error = "cannot open blocked-cell image: " + path;
    return false;
  }
  const std::string bytes(
    (std::istreambuf_iterator<char>(stream)), std::istreambuf_iterator<char>());
  if (bytes.size() < 8) {
    error = "blocked-cell image is truncated: " + path;
    return false;
  }

  std::size_t cursor = 0;
  std::string token;
  std::string magic;
  if (!read_numeric_token(bytes, cursor, magic) || (magic != "P5" && magic != "P2")) {
    error = "blocked-cell image is not a binary or ascii PGM: " + path;
    return false;
  }

  long width = 0;
  long height = 0;
  long ceiling = 0;
  if (!read_numeric_token(bytes, cursor, token) || !parse_positive(token, width) ||
    !read_numeric_token(bytes, cursor, token) || !parse_positive(token, height) ||
    !read_numeric_token(bytes, cursor, token) || !parse_positive(token, ceiling) ||
    ceiling > 65535)
  {
    error = "blocked-cell image header is malformed: " + path;
    return false;
  }

  columns = static_cast<int>(width);
  rows = static_cast<int>(height);
  max_value = static_cast<int>(ceiling);
  const std::size_t count = static_cast<std::size_t>(columns) * rows;
  raster_values.assign(count, 0);

  if (magic == "P5") {
    if (cursor < bytes.size() && bytes[cursor] == '\r') {
      ++cursor;
    }
    if (cursor < bytes.size() && std::isspace(static_cast<unsigned char>(bytes[cursor])) != 0) {
      ++cursor;
    }
    const std::size_t bytes_per_sample = max_value > 255 ? 2U : 1U;
    if (bytes.size() - cursor < count * bytes_per_sample) {
      error = "blocked-cell image payload is truncated: " + path;
      return false;
    }
    if (bytes_per_sample == 1) {
      for (std::size_t index = 0; index < count; ++index) {
        raster_values[index] = static_cast<uint16_t>(static_cast<unsigned char>(bytes[cursor + index]));
      }
    } else {
      for (std::size_t index = 0; index < count; ++index) {
        const unsigned char high = static_cast<unsigned char>(bytes[cursor + 2 * index]);
        const unsigned char low = static_cast<unsigned char>(bytes[cursor + 2 * index + 1]);
        raster_values[index] = static_cast<uint16_t>((high << 8) | low);
      }
    }
  } else {
    for (std::size_t index = 0; index < count; ++index) {
      if (!read_numeric_token(bytes, cursor, token)) {
        error = "blocked-cell image payload is truncated: " + path;
        return false;
      }
      try {
        const long sample = std::stol(token);
        if (sample < 0 || sample > ceiling) {
          error = "blocked-cell image sample is out of range: " + path;
          return false;
        }
        raster_values[index] = static_cast<uint16_t>(sample);
      } catch (const std::exception &) {
        error = "blocked-cell image sample is not a number: " + path;
        return false;
      }
    }
  }
  return true;
}

bool PlanAtlas::load(
  const std::string & descriptor_path, double affinity_sigma, std::string & error)
{
  Descriptor descriptor;
  if (!read_descriptor(descriptor_path, descriptor, error)) {
    return false;
  }

  int columns = 0;
  int rows = 0;
  int max_value = 0;
  std::vector<uint16_t> samples;
  if (!read_pgm(descriptor.raster_path, columns, rows, max_value, samples, error)) {
    return false;
  }
  if (max_value <= 0) {
    error = "blocked-cell image has a zero maximum value: " + descriptor.raster_path;
    return false;
  }

  std::vector<uint8_t> mask(static_cast<std::size_t>(columns) * rows, 0);
  const double scale = 1.0 / static_cast<double>(max_value);
  for (int row = 0; row < rows; ++row) {
    const int source_row = rows - 1 - row;
    for (int column = 0; column < columns; ++column) {
      const uint16_t sample = samples[static_cast<std::size_t>(source_row) * columns + column];
      const double blocked_probability = descriptor.negated ?
        static_cast<double>(sample) * scale :
        (static_cast<double>(max_value) - sample) * scale;
      if (blocked_probability >= descriptor.blocked_threshold) {
        mask[static_cast<std::size_t>(row) * columns + column] = 1;
      }
    }
  }

  if (!adopt(
      columns, rows, descriptor.cell_span, descriptor.anchor_x, descriptor.anchor_y,
      descriptor.anchor_heading, std::move(mask), affinity_sigma, error))
  {
    return false;
  }
  raster_path_ = descriptor.raster_path;
  digest_ = digest_of_file(descriptor.raster_path);
  return true;
}

bool PlanAtlas::adopt(
  int columns, int rows, double cell_span, double anchor_x, double anchor_y,
  double anchor_heading, std::vector<uint8_t> blocked, double affinity_sigma,
  std::string & error)
{
  if (columns <= 0 || rows <= 0) {
    error = "plan dimensions must be positive";
    return false;
  }
  if (!(cell_span > 0.0)) {
    error = "plan resolution must be positive";
    return false;
  }
  if (blocked.size() != static_cast<std::size_t>(columns) * rows) {
    error = "plan mask has the wrong number of cells";
    return false;
  }
  if (!(affinity_sigma > 0.0)) {
    error = "affinity width must be positive";
    return false;
  }

  columns_ = columns;
  rows_ = rows;
  cell_span_ = cell_span;
  inverse_span_ = 1.0 / cell_span;
  anchor_x_ = anchor_x;
  anchor_y_ = anchor_y;
  anchor_heading_ = wrap_heading(anchor_heading);
  axis_aligned_ = std::abs(anchor_heading_) < 1.0e-12;
  blocked_ = std::move(blocked);
  affinity_sigma_ = affinity_sigma;
  return rebuild_fields(affinity_sigma, error);
}

bool PlanAtlas::rebuild_fields(double affinity_sigma, std::string & error)
{
  std::vector<float> squared;
  squared_distance_field(blocked_, columns_, rows_, squared);

  clearance_.resize(squared.size());
  affinity_.resize(squared.size());
  const double inverse_two_sigma_squared = 1.0 / (2.0 * affinity_sigma * affinity_sigma);
  for (std::size_t index = 0; index < squared.size(); ++index) {
    const double steps = std::sqrt(static_cast<double>(squared[index]));
    const double metric = steps * cell_span_;
    clearance_[index] = static_cast<float>(metric);
    affinity_[index] = static_cast<float>(std::exp(-metric * metric * inverse_two_sigma_squared));
  }
  (void)error;
  return true;
}

double PlanAtlas::clearance_at(double wx, double wy) const
{
  int column = 0;
  int row = 0;
  if (!locate(wx, wy, column, row)) {
    return -1.0;
  }
  return clearance_[static_cast<std::size_t>(row) * columns_ + column];
}

Eigen::Vector2d PlanAtlas::cell_corner(int column, int row) const
{
  const double local_x = column * cell_span_;
  const double local_y = row * cell_span_;
  if (axis_aligned_) {
    return Eigen::Vector2d(anchor_x_ + local_x, anchor_y_ + local_y);
  }
  const double c = std::cos(anchor_heading_);
  const double s = std::sin(anchor_heading_);
  return Eigen::Vector2d(
    anchor_x_ + c * local_x - s * local_y,
    anchor_y_ + s * local_x + c * local_y);
}

}  // namespace relocalizer
