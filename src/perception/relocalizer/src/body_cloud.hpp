#pragma once

#include <algorithm>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <string>

#include <rcpputils/endian.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/LinearMath/Transform.h>

namespace relocalizer
{
// Convert a filtered float32 XYZ cloud to compact, finite body-frame points.
inline sensor_msgs::msg::PointCloud2 body_cloud(
  const sensor_msgs::msg::PointCloud2 & input, const tf2::Transform & to_body,
  const std::string & frame)
{
  using Field = sensor_msgs::msg::PointField;
  const auto offset = [&](const char * name) {
    const auto field = std::find_if(input.fields.begin(), input.fields.end(),
      [name](const Field & f) { return f.name == name; });
    if (field == input.fields.end() || field->datatype != Field::FLOAT32 ||
        field->count != 1 || size_t{field->offset} + sizeof(float) > input.point_step) {
      throw std::invalid_argument(std::string("Expected float32 field: ") + name);
    }
    return field->offset;
  };
  const auto x = offset("x"), y = offset("y"), z = offset("z");
  if (size_t{input.width} * input.point_step > input.row_step ||
      size_t{input.height} * input.row_step > input.data.size()) {
    throw std::invalid_argument("Incomplete PointCloud2 rows");
  }

  sensor_msgs::msg::PointCloud2 output;
  output.header = input.header;
  output.header.frame_id = frame;
  output.height = 1;
  output.is_dense = true;
  output.is_bigendian = rcpputils::endian::native == rcpputils::endian::big;
  sensor_msgs::PointCloud2Modifier modifier(output);
  modifier.setPointCloud2Fields(3, "x", 1, Field::FLOAT32, "y", 1, Field::FLOAT32,
    "z", 1, Field::FLOAT32);
  if (input.width == 0 || input.height == 0) {
    return output;
  }
  modifier.resize(size_t{input.width} * input.height);
  const auto read = [swap = input.is_bigendian != output.is_bigendian](const uint8_t * bytes) {
    float value;
    auto * dest = reinterpret_cast<uint8_t *>(&value);
    if (swap) {
      std::reverse_copy(bytes, bytes + sizeof(float), dest);
    } else {
      std::memcpy(dest, bytes, sizeof(float));
    }
    return value;
  };

  sensor_msgs::PointCloud2Iterator<float> xyz(output, "x");
  size_t count = 0;
  for (uint32_t row = 0; row < input.height; ++row) {
    const uint8_t * point = input.data.data() + size_t{row} * input.row_step;
    for (uint32_t col = 0; col < input.width; ++col, point += input.point_step) {
      const tf2::Vector3 v(read(point + x), read(point + y), read(point + z));
      if (std::isfinite(v.x()) && std::isfinite(v.y()) && std::isfinite(v.z())) {
        const auto transformed = to_body * v;
        xyz[0] = transformed.x();
        xyz[1] = transformed.y();
        xyz[2] = transformed.z();
        ++xyz;
        ++count;
      }
    }
  }
  modifier.resize(count);
  return output;
}
}  // namespace relocalizer
