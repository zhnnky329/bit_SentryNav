#include <gtest/gtest.h>

#include <limits>

#include "../src/body_cloud.hpp"

TEST(BodyCloud, TransformsXYZAndKeepsLidarStamp)
{
  sensor_msgs::msg::PointCloud2 input;
  input.height = 1;
  input.header.frame_id = "lidar";
  input.header.stamp.sec = 12;
  input.header.stamp.nanosec = 34;
  sensor_msgs::PointCloud2Modifier modifier(input);
  using Field = sensor_msgs::msg::PointField;
  modifier.setPointCloud2Fields(3, "z", 1, Field::FLOAT32, "x", 1, Field::FLOAT32,
    "y", 1, Field::FLOAT32);
  modifier.resize(3);
  sensor_msgs::PointCloud2Iterator<float> zxy(input, "z");
  zxy[0] = 4; zxy[1] = 2; zxy[2] = 3;
  ++zxy;
  zxy[1] = std::numeric_limits<float>::quiet_NaN();
  ++zxy;
  zxy[0] = 0; zxy[1] = 1; zxy[2] = 0;
  // Turn the three points into rows with four bytes of padding after each row.
  input.width = 1;
  input.height = 3;
  input.row_step = 16;
  for (size_t i : {36, 24, 12}) {
    input.data.insert(input.data.begin() + i, 4, 0);
  }
  tf2::Quaternion rotation;
  rotation.setRPY(0., 0., std::acos(-1.) / 2);
  const tf2::Transform to_body(rotation, tf2::Vector3(1, -2, 0.5));
  const auto output = relocalizer::body_cloud(input, to_body, "base_link");
  EXPECT_EQ(output.header.frame_id, "base_link");
  EXPECT_EQ(output.header.stamp, input.header.stamp);
  ASSERT_EQ(output.width, 2u);
  EXPECT_EQ(output.height, 1u);
  sensor_msgs::PointCloud2ConstIterator<float> xyz(output, "x");
  EXPECT_NEAR(xyz[0], -2, 1e-6);
  EXPECT_NEAR(xyz[1], 0, 1e-6);
  EXPECT_NEAR(xyz[2], 4.5, 1e-6);
  ++xyz;
  EXPECT_NEAR(xyz[0], 1, 1e-6);
  EXPECT_NEAR(xyz[1], -1, 1e-6);
  EXPECT_NEAR(xyz[2], 0.5, 1e-6);
}
