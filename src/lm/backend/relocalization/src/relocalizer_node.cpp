// Copyright 2026 bw_nav contributors
// SPDX-License-Identifier: Apache-2.0
//
// ROS 2 wrapper around the relocalization engine.  Consumes a body-frame
// point surface, maintains the world-to-body transform, and exposes a trigger
// service for an explicit search.

#include <chrono>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Core>

#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include "relocalizer/engine.hpp"

namespace relocalizer
{
namespace
{

std::string format(double value, int digits)
{
  std::ostringstream stream;
  stream.setf(std::ios::fixed);
  stream.precision(digits);
  stream << value;
  return stream.str();
}

}  // namespace

class RelocalizerNode : public rclcpp::Node
{
public:
  RelocalizerNode()
  : rclcpp::Node("relocalizer")
  {
    load_parameters();

    std::string error;
    if (!engine_.configure(settings_, error)) {
      throw std::runtime_error("cannot load maps: " + error);
    }

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    if (!odom_frame_.empty()) {
      tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
      tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    }

    pose_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>("~/pose", 10);
    report_publisher_ = create_publisher<std_msgs::msg::String>("~/status", 10);
    proposals_publisher_ =
      create_publisher<geometry_msgs::msg::PoseArray>("~/proposals", 10);
    plan_layer_publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      "~/plan", rclcpp::QoS(1).transient_local().reliable());
    surface_layer_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "~/surface_map", rclcpp::QoS(1).transient_local().reliable());
    publish_plan_layer();
    publish_surface_layer();

    sweep_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      sweep_topic_, rclcpp::SensorDataQoS(),
      std::bind(&RelocalizerNode::on_sweep, this, std::placeholders::_1));
    initial_subscription_ =
      create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose", 10,
      std::bind(&RelocalizerNode::on_initial_pose, this, std::placeholders::_1));
    relocalize_service_ = create_service<std_srvs::srv::Trigger>(
      "~/relocalize",
      std::bind(
        &RelocalizerNode::on_relocalize, this, std::placeholders::_1,
        std::placeholders::_2));

    if (initial_pose_.size() == 3) {
      pose_.x = initial_pose_[0];
      pose_.y = initial_pose_[1];
      pose_.heading = wrap_heading(deg_to_rad(initial_pose_[2]));
      phase_ = "PRIOR";
    }

    RCLCPP_INFO(
      get_logger(), "plan %dx%d, surface %zu points, %d voxel tiers; sweep '%s'",
      engine_.plan().columns(), engine_.plan().rows(), engine_.atlas().point_count(),
      engine_.atlas().tier_count(), sweep_topic_.c_str());
  }

private:
  void load_parameters()
  {
    settings_.plan_descriptor = declare_parameter<std::string>("plan_descriptor", "");
    settings_.surface_map = declare_parameter<std::string>("surface_map", "");
    if (settings_.plan_descriptor.empty() || settings_.surface_map.empty()) {
      throw std::runtime_error("'plan_descriptor' and 'surface_map' must be provided");
    }
    world_frame_ = declare_parameter<std::string>("world_frame", "map");
    body_frame_ = declare_parameter<std::string>("body_frame", "base_link");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "");
    sweep_topic_ = declare_parameter<std::string>("sweep_topic", "/cloud_registered_body");
    spark_height_ = declare_parameter<double>("spark_height", 0.0);
    automatic_ = declare_parameter<bool>("automatic", true);
    search_period_ = declare_parameter<double>("search_period_sec", 1.5);
    missed_limit_ = declare_parameter<int>("missed_limit", 4);
    minimum_tracking_coherence_ = declare_parameter<double>("minimum_tracking_coherence", 0.35);
    minimum_tracking_overlap_ = declare_parameter<double>("minimum_tracking_overlap", 0.35);
    initial_pose_ = declare_parameter<std::vector<double>>("initial_pose", std::vector<double>{});

    settings_.affinity_sigma = declare_parameter<double>("affinity_sigma", 0.50);
    settings_.voxel_spans = declare_parameter<std::vector<double>>(
      "voxel_spans", std::vector<double>{2.0, 1.0});
    settings_.voxel_floor = declare_parameter<double>("voxel_floor", 0.25);
    settings_.voxel_ceiling = declare_parameter<double>("voxel_ceiling", 3.00);
    settings_.bag_span = declare_parameter<double>("bag_span", 0.50);
    settings_.bag_radius = declare_parameter<double>("bag_radius", 0.80);

    settings_.sweep.search_floor = declare_parameter<double>("sweep_search_floor", 0.35);
    settings_.sweep.search_ceiling = declare_parameter<double>("sweep_search_ceiling", 2.20);
    settings_.sweep.core_floor = declare_parameter<double>("sweep_core_floor", 0.30);
    settings_.sweep.core_ceiling = declare_parameter<double>("sweep_core_ceiling", 2.30);
    settings_.sweep.nearest_reach = declare_parameter<double>("sweep_nearest_reach", 0.80);
    settings_.sweep.farthest_reach = declare_parameter<double>("sweep_farthest_reach", 60.0);
    settings_.sweep.search_cell = declare_parameter<double>("sweep_search_cell", 0.15);
    settings_.sweep.search_limit = declare_parameter<int>("sweep_search_limit", 800);
    settings_.sweep.core_cell = declare_parameter<double>("sweep_core_cell", 0.20);
    settings_.sweep.core_limit = declare_parameter<int>("sweep_core_limit", 1500);

    settings_.align.max_iterations = declare_parameter<int>("align_max_iterations", 40);
    settings_.align.step_tolerance = declare_parameter<double>("align_step_tolerance", 1.0e-4);
    settings_.align.maximum_shift = declare_parameter<double>("align_maximum_shift", 0.8);
    settings_.align.maximum_turn = declare_parameter<double>("align_maximum_turn", 0.5);
    settings_.align.minimum_overlap = declare_parameter<int>("align_minimum_overlap", 25);

    settings_.search.heading_step_deg =
      declare_parameter<double>("search_heading_step_deg", 10.0);
    settings_.search.coarse_step = declare_parameter<double>("search_coarse_step", 0.40);
    settings_.search.coarse_floor = declare_parameter<double>("search_coarse_floor", 0.45);
    settings_.search.max_positions = declare_parameter<int>("search_max_positions", 22000);
    settings_.search.coarse_keep = declare_parameter<int>("search_coarse_keep", 200);
    settings_.search.fine_step = declare_parameter<double>("search_fine_step", 0.20);
    settings_.search.fine_heading_deg =
      declare_parameter<double>("search_fine_heading_deg", 2.0);
    settings_.search.fine_floor = declare_parameter<double>("search_fine_floor", 0.55);
    settings_.search.fine_keep = declare_parameter<int>("search_fine_keep", 24);
    settings_.search.place_separation =
      declare_parameter<double>("search_place_separation", 1.0);
    settings_.search.keep_hypotheses =
      declare_parameter<int>("search_keep_hypotheses", 12);
    settings_.search.threads = declare_parameter<int>("search_threads", 0);

    settings_.quality.surface_radius =
      declare_parameter<double>("quality_surface_radius", 0.30);
    settings_.quality.nearest_limit =
      declare_parameter<double>("quality_nearest_limit", 0.80);
    settings_.quality.open_clearance =
      declare_parameter<double>("quality_open_clearance", 0.60);
    settings_.minimum_surface_ratio =
      declare_parameter<double>("minimum_surface_ratio", 0.62);
    settings_.minimum_surface_points =
      declare_parameter<int>("minimum_surface_points", 60);
    settings_.maximum_open_ratio = declare_parameter<double>("maximum_open_ratio", 0.30);
    settings_.minimum_pose_clearance =
      declare_parameter<double>("minimum_pose_clearance", 0.05);
    settings_.ambiguity_margin = declare_parameter<double>("ambiguity_margin", 0.08);
  }

  void on_sweep(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & message)
  {
    if (message->data.empty() || message->width * message->height == 0) {
      return;
    }
    std::vector<Eigen::Vector3d> surface;
    surface.reserve(static_cast<std::size_t>(message->width) * message->height);
    try {
      sensor_msgs::PointCloud2ConstIterator<float> x(*message, "x");
      sensor_msgs::PointCloud2ConstIterator<float> y(*message, "y");
      sensor_msgs::PointCloud2ConstIterator<float> z(*message, "z");
      for (; x != x.end(); ++x, ++y, ++z) {
        const double px = static_cast<double>(*x);
        const double py = static_cast<double>(*y);
        const double pz = static_cast<double>(*z);
        if (!std::isfinite(px) || !std::isfinite(py) || !std::isfinite(pz)) {
          continue;
        }
        if (px * px + py * py > 100.0 * 100.0) {
          continue;
        }
        surface.emplace_back(px, py, pz);
        if (surface.size() >= 250000) {
          break;
        }
      }
    } catch (const std::exception & exception) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "cannot read sweep fields: %s", exception.what());
      return;
    }
    if (surface.size() < 200) {
      return;
    }

    last_sweep_ = surface;
    last_stamp_ = message->header.stamp;
    const rclcpp::Time now = this->now();
    const SweepSlice slice = reduce_sweep(surface, engine_.settings().sweep);
    if (!slice.usable()) {
      return;
    }

    if (phase_ == "TRACKING" || phase_ == "PRIOR") {
      const AlignReport aligned = engine_.refine(slice.core_points, pose_);
      const Quality quality = inspect_pose(
        engine_.plan(), engine_.atlas(), slice.core_points, aligned.pose,
        engine_.settings().quality);
      const bool verified = aligned.overlap >= minimum_tracking_overlap_ &&
        aligned.coherence >= minimum_tracking_coherence_ &&
        quality.surface_ratio >= 0.5 * engine_.settings().minimum_surface_ratio;
      if (verified) {
        pose_ = aligned.pose;
        phase_ = "TRACKING";
        misses_ = 0;
        publish_pose(last_stamp_);
        publish_status(aligned.coherence, "tracking");
        return;
      }
      ++misses_;
      if (misses_ < missed_limit_) {
        return;
      }
      phase_ = "LOST";
      RCLCPP_WARN(get_logger(), "tracking lost after %d rejected sweeps", misses_);
      if (!automatic_) {
        publish_status(0.0, "lost");
        return;
      }
    }

    if (!automatic_) {
      publish_status(0.0, "waiting for a relocalize request");
      return;
    }
    if (now - last_search_time_ < rclcpp::Duration::from_seconds(search_period_)) {
      return;
    }
    run_search(surface, last_stamp_);
  }

  void on_initial_pose(
    const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr & message)
  {
    pose_.x = message->pose.pose.position.x;
    pose_.y = message->pose.pose.position.y;
    const auto & rotation = message->pose.pose.orientation;
    pose_.heading = std::atan2(
      2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
      1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z));
    phase_ = "PRIOR";
    misses_ = 0;
    RCLCPP_INFO(
      get_logger(), "external prior at %.2f %.2f %.1f deg", pose_.x, pose_.y,
      rad_to_deg(pose_.heading));
  }

  bool on_relocalize(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> reply)
  {
    if (last_sweep_.empty()) {
      reply->success = false;
      reply->message = "no sweep received yet";
      return true;
    }
    const Outcome outcome = engine_.relocalize(last_sweep_);
    last_search_time_ = this->now();
    if (outcome.confirmed) {
      pose_ = outcome.pose;
      phase_ = "TRACKING";
      misses_ = 0;
      publish_pose(last_stamp_);
      publish_status(outcome.confidence, outcome.explanation);
    } else {
      publish_status(0.0, outcome.explanation);
    }
    reply->success = outcome.confirmed;
    reply->message = outcome.explanation;
    return true;
  }

  void run_search(
    const std::vector<Eigen::Vector3d> & surface, const builtin_interfaces::msg::Time & stamp)
  {
    last_search_time_ = this->now();
    const Outcome outcome = engine_.relocalize(surface);
    publish_proposals(outcome.proposals, stamp);
    if (!outcome.confirmed) {
      phase_ = (phase_ == "PRIOR") ? "PRIOR" : "LOST";
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000, "search rejected: %s", outcome.explanation.c_str());
      publish_status(0.0, outcome.explanation);
      return;
    }
    pose_ = outcome.pose;
    phase_ = "TRACKING";
    misses_ = 0;
    RCLCPP_INFO(
      get_logger(), "localized at %.2f %.2f %.1f deg (confidence %.2f)",
      pose_.x, pose_.y, rad_to_deg(pose_.heading), outcome.confidence);
    publish_pose(stamp);
    publish_status(outcome.confidence, outcome.explanation);
  }

  void publish_plan_layer()
  {
    const PlanAtlas & plan = engine_.plan();
    nav_msgs::msg::OccupancyGrid message;
    message.header.stamp = this->now();
    message.header.frame_id = world_frame_;
    message.info.resolution = static_cast<float>(plan.cell_span());
    message.info.width = static_cast<uint32_t>(plan.columns());
    message.info.height = static_cast<uint32_t>(plan.rows());
    message.info.origin.position.x = plan.anchor_x();
    message.info.origin.position.y = plan.anchor_y();
    message.info.origin.position.z = 0.0;
    tf2::Quaternion rotation;
    rotation.setRPY(0.0, 0.0, plan.anchor_heading());
    message.info.origin.orientation.x = rotation.x();
    message.info.origin.orientation.y = rotation.y();
    message.info.origin.orientation.z = rotation.z();
    message.info.origin.orientation.w = rotation.w();
    message.data.resize(static_cast<std::size_t>(plan.columns()) * plan.rows());
    const std::vector<uint8_t> & blocked = plan.blocked();
    for (std::size_t index = 0; index < blocked.size(); ++index) {
      message.data[index] = blocked[index] ? 100 : 0;
    }
    plan_layer_publisher_->publish(message);
  }

  void publish_surface_layer()
  {
    const std::vector<Eigen::Vector3d> & samples = engine_.atlas().samples();
    sensor_msgs::msg::PointCloud2 message;
    message.header.stamp = this->now();
    message.header.frame_id = world_frame_;
    message.height = 1;
    message.width = static_cast<uint32_t>(samples.size());
    message.fields.resize(3);
    const char * names[3] = {"x", "y", "z"};
    for (std::size_t axis = 0; axis < 3; ++axis) {
      message.fields[axis].name = names[axis];
      message.fields[axis].offset = static_cast<uint32_t>(axis * sizeof(float));
      message.fields[axis].datatype = sensor_msgs::msg::PointField::FLOAT32;
      message.fields[axis].count = 1;
    }
    message.point_step = static_cast<uint32_t>(3 * sizeof(float));
    message.row_step = message.point_step * message.width;
    message.is_bigendian = false;
    message.is_dense = true;
    message.data.resize(static_cast<std::size_t>(message.row_step));
    for (std::size_t index = 0; index < samples.size(); ++index) {
      const float coordinates[3] = {
        static_cast<float>(samples[index].x()),
        static_cast<float>(samples[index].y()),
        static_cast<float>(samples[index].z())};
      std::memcpy(
        &message.data[index * message.point_step], coordinates, sizeof(coordinates));
    }
    surface_layer_publisher_->publish(message);
  }

  void publish_pose(const builtin_interfaces::msg::Time & stamp)
  {
    geometry_msgs::msg::PoseStamped message;
    message.header.stamp = stamp;
    message.header.frame_id = world_frame_;
    message.pose.position.x = pose_.x;
    message.pose.position.y = pose_.y;
    message.pose.position.z = spark_height_;
    tf2::Quaternion rotation;
    rotation.setRPY(0.0, 0.0, pose_.heading);
    message.pose.orientation.x = rotation.x();
    message.pose.orientation.y = rotation.y();
    message.pose.orientation.z = rotation.z();
    message.pose.orientation.w = rotation.w();
    pose_publisher_->publish(message);
    broadcast_transform(stamp);
  }

  void broadcast_transform(const builtin_interfaces::msg::Time & stamp)
  {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = stamp;
    transform.header.frame_id = world_frame_;
    transform.child_frame_id = body_frame_;
    transform.transform.translation.x = pose_.x;
    transform.transform.translation.y = pose_.y;
    transform.transform.translation.z = spark_height_;
    tf2::Quaternion rotation;
    rotation.setRPY(0.0, 0.0, pose_.heading);
    transform.transform.rotation.x = rotation.x();
    transform.transform.rotation.y = rotation.y();
    transform.transform.rotation.z = rotation.z();
    transform.transform.rotation.w = rotation.w();

    if (odom_frame_.empty()) {
      tf_broadcaster_->sendTransform(transform);
      return;
    }

    try {
      const geometry_msgs::msg::TransformStamped odom_to_body = tf_buffer_->lookupTransform(
        odom_frame_, body_frame_, stamp, tf2::durationFromSec(0.05));
      const auto & translation = odom_to_body.transform.translation;
      const auto & orientation = odom_to_body.transform.rotation;
      Placement odom_pose;
      odom_pose.x = translation.x;
      odom_pose.y = translation.y;
      odom_pose.heading = std::atan2(
        2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
        1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z));
      const Placement world_to_odom = compose(pose_, invert(odom_pose));
      transform.header.frame_id = world_frame_;
      transform.child_frame_id = odom_frame_;
      transform.transform.translation.x = world_to_odom.x;
      transform.transform.translation.y = world_to_odom.y;
      transform.transform.translation.z = 0.0;
      tf2::Quaternion odom_rotation;
      odom_rotation.setRPY(0.0, 0.0, world_to_odom.heading);
      transform.transform.rotation.x = odom_rotation.x();
      transform.transform.rotation.y = odom_rotation.y();
      transform.transform.rotation.z = odom_rotation.z();
      transform.transform.rotation.w = odom_rotation.w();
      tf_broadcaster_->sendTransform(transform);
    } catch (const tf2::TransformException & exception) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "cannot build world->odom: %s", exception.what());
    }
  }

  void publish_proposals(
    const std::vector<Hypothesis> & proposals, const builtin_interfaces::msg::Time & stamp)
  {
    geometry_msgs::msg::PoseArray message;
    message.header.stamp = stamp;
    message.header.frame_id = world_frame_;
    message.poses.reserve(proposals.size());
    for (const Hypothesis & proposal : proposals) {
      geometry_msgs::msg::Pose pose;
      pose.position.x = proposal.pose.x;
      pose.position.y = proposal.pose.y;
      pose.position.z = spark_height_;
      tf2::Quaternion rotation;
      rotation.setRPY(0.0, 0.0, proposal.pose.heading);
      pose.orientation.x = rotation.x();
      pose.orientation.y = rotation.y();
      pose.orientation.z = rotation.z();
      pose.orientation.w = rotation.w();
      message.poses.push_back(pose);
    }
    proposals_publisher_->publish(message);
  }

  void publish_status(double confidence, const std::string & note)
  {
    std_msgs::msg::String message;
    std::ostringstream stream;
    stream << "{\"state\":\"" << phase_ << "\",\"confidence\":" << format(confidence, 3)
           << ",\"x\":" << format(pose_.x, 3) << ",\"y\":" << format(pose_.y, 3)
           << ",\"heading_deg\":" << format(rad_to_deg(pose_.heading), 2)
           << ",\"note\":\"" << note << "\"}";
    message.data = stream.str();
    report_publisher_->publish(message);
  }

  EngineSettings settings_;
  Engine engine_;
  std::string world_frame_;
  std::string body_frame_;
  std::string odom_frame_;
  std::string sweep_topic_;
  double spark_height_{0.0};
  bool automatic_{true};
  double search_period_{1.5};
  int missed_limit_{4};
  double minimum_tracking_coherence_{0.35};
  double minimum_tracking_overlap_{0.35};
  std::vector<double> initial_pose_;

  Placement pose_;
  std::string phase_{"LOST"};
  int misses_{0};
  std::vector<Eigen::Vector3d> last_sweep_;
  builtin_interfaces::msg::Time last_stamp_;
  rclcpp::Time last_search_time_{0, 0, RCL_ROS_TIME};

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr report_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr proposals_publisher_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr plan_layer_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr surface_layer_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sweep_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    initial_subscription_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr relocalize_service_;
};

}  // namespace relocalizer

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<relocalizer::RelocalizerNode>();
    rclcpp::spin(node);
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(rclcpp::get_logger("relocalizer"), "%s", exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
