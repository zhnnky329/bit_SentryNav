#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include <chrono>
#include <cmath>
#include <deque>
#include <mutex>

class TfOnlyOdom : public rclcpp::Node
{
public:
    TfOnlyOdom()
        : Node("tf_only_odom")
    {
        // 参数：base_link → livox_frame 的变换 C
        this->declare_parameter<double>("base_link_to_livox_x", 0.0);
        this->declare_parameter<double>("base_link_to_livox_y", 0.0);
        this->declare_parameter<double>("base_link_to_livox_z", 0.0);
        this->declare_parameter<double>("base_link_to_livox_roll", 0.0);
        this->declare_parameter<double>("base_link_to_livox_pitch", 0.0);
        this->declare_parameter<double>("base_link_to_livox_yaw", 0.0);
        this->declare_parameter<double>("publish_rate", 50.0);
        this->declare_parameter<bool>("align_base_link_to_map_on_start", true);
        // false 时由 relocalizer 发布 map->odom，本节点只发布 odom->base_link
        this->declare_parameter<bool>("publish_map_to_odom", true);

        // frame 名称参数（用于 map->odom 静态变换）
        this->declare_parameter<std::string>("map_frame_for_A", "map");
        this->declare_parameter<std::string>("odom_frame", "odom");

        // map -> odom 静态修正（建图起点与本次启动位姿不一致时使用）
        this->declare_parameter<double>("map_to_odom_x", 0.0);
        this->declare_parameter<double>("map_to_odom_y", 0.0);
        this->declare_parameter<double>("map_to_odom_z", 0.0);
        this->declare_parameter<double>("map_to_odom_roll", 0.0);
        this->declare_parameter<double>("map_to_odom_pitch", 0.0);
        this->declare_parameter<double>("map_to_odom_yaw", 0.0);

        // 读取参数
        this->get_parameter("base_link_to_livox_x", livox_offset_x_);
        this->get_parameter("base_link_to_livox_y", livox_offset_y_);
        this->get_parameter("base_link_to_livox_z", livox_offset_z_);

        double roll_deg = 0.0, pitch_deg = 0.0, yaw_deg = 0.0;
        this->get_parameter("base_link_to_livox_roll", roll_deg);
        this->get_parameter("base_link_to_livox_pitch", pitch_deg);
        this->get_parameter("base_link_to_livox_yaw", yaw_deg);

        livox_offset_roll_  = roll_deg * M_PI / 180.0;
        livox_offset_pitch_ = pitch_deg * M_PI / 180.0;
        livox_offset_yaw_   = yaw_deg * M_PI / 180.0;

        double publish_rate = 100.0;
        this->get_parameter("publish_rate", publish_rate);
        this->get_parameter("align_base_link_to_map_on_start", align_base_link_to_map_on_start_);
        this->get_parameter("publish_map_to_odom", publish_map_to_odom_);

        this->get_parameter("map_frame_for_A", map_frame_for_A_);
        this->get_parameter("odom_frame", odom_frame_);

        double map_to_odom_roll_deg = 0.0;
        double map_to_odom_pitch_deg = 0.0;
        double map_to_odom_yaw_deg = 0.0;
        this->get_parameter("map_to_odom_x", map_to_odom_x_);
        this->get_parameter("map_to_odom_y", map_to_odom_y_);
        this->get_parameter("map_to_odom_z", map_to_odom_z_);
        this->get_parameter("map_to_odom_roll", map_to_odom_roll_deg);
        this->get_parameter("map_to_odom_pitch", map_to_odom_pitch_deg);
        this->get_parameter("map_to_odom_yaw", map_to_odom_yaw_deg);
        map_to_odom_roll_ = map_to_odom_roll_deg * M_PI / 180.0;
        map_to_odom_pitch_ = map_to_odom_pitch_deg * M_PI / 180.0;
        map_to_odom_yaw_ = map_to_odom_yaw_deg * M_PI / 180.0;

        // 计算 C: base_link → livox_frame
        tf_C_.setOrigin(tf2::Vector3(livox_offset_x_, livox_offset_y_, livox_offset_z_));
        tf2::Quaternion q_C;
        q_C.setRPY(livox_offset_roll_, livox_offset_pitch_, livox_offset_yaw_);
        tf_C_.setRotation(q_C);
        tf_C_inv_ = tf_C_.inverse();

        // 计算 A: map → odom
        tf_A_.setOrigin(tf2::Vector3(map_to_odom_x_, map_to_odom_y_, map_to_odom_z_));
        tf2::Quaternion q_A;
        q_A.setRPY(map_to_odom_roll_, map_to_odom_pitch_, map_to_odom_yaw_);
        tf_A_.setRotation(q_A);

        static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 50);

        // 订阅 /lio/robo/odom 用于构造 E(odom->livox_frame_two)
        lio_odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/lio/robo/odom",
            rclcpp::QoS(rclcpp::KeepLast(200)).best_effort().durability_volatile(),
            std::bind(&TfOnlyOdom::lioOdomCallback, this, std::placeholders::_1));

        // RViz 的 2D Pose Estimate：手动指定 base_link 在 map 中的位姿，
        // 节点据此反算并更新 map->odom。使用 relocalizer 时关闭。
        if (publish_map_to_odom_) {
            initial_pose_subscriber_ =
                this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
                    "/initialpose",
                    rclcpp::QoS(10),
                    std::bind(&TfOnlyOdom::initialPoseCallback, this, std::placeholders::_1));
        }

        auto period = std::chrono::duration<double>(1.0 / publish_rate);
        timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::milliseconds>(period),
            std::bind(&TfOnlyOdom::timerCallback, this));

        publishStaticTransforms();

        if (publish_map_to_odom_) {
            RCLCPP_INFO(
                this->get_logger(),
                "已发布静态 C(base_link->livox_frame)；A(%s->%s) 动态发布，可由 /initialpose 更新。",
                map_frame_for_A_.c_str(), odom_frame_.c_str());
            RCLCPP_INFO(
                this->get_logger(),
                "map->odom 修正: x=%.3f y=%.3f z=%.3f roll=%.2f pitch=%.2f yaw=%.2f deg",
                map_to_odom_x_, map_to_odom_y_, map_to_odom_z_,
                map_to_odom_roll_ * 180.0 / M_PI,
                map_to_odom_pitch_ * 180.0 / M_PI,
                map_to_odom_yaw_ * 180.0 / M_PI);
        } else {
            RCLCPP_INFO(
                this->get_logger(),
                "已发布静态 C(base_link->livox_frame)；map->odom 由 relocalizer 发布，本节点不发布 A。");
        }
        RCLCPP_INFO(this->get_logger(),
                "align_base_link_to_map_on_start = %s",
                align_base_link_to_map_on_start_ ? "true" : "false");
    }

private:
    void publishStaticTransforms()
    {
        // 静态 C: base_link -> livox_frame
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = rclcpp::Time(0);
        tf_msg.header.frame_id = "base_link";
        tf_msg.child_frame_id = "livox_frame";
        tf_msg.transform = tf2::toMsg(tf_C_);
        static_tf_broadcaster_->sendTransform(tf_msg);
        RCLCPP_INFO(this->get_logger(), "静态变换已发布: base_link -> livox_frame");
    }

    void initialPoseCallback(
        const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        if (!msg) return;
        if (!msg->header.frame_id.empty() && msg->header.frame_id != map_frame_for_A_) {
            RCLCPP_WARN(this->get_logger(),
                        "/initialpose frame_id='%s'，按 '%s' 处理。",
                        msg->header.frame_id.c_str(), map_frame_for_A_.c_str());
        }

        tf2::Transform T_map_base;
        T_map_base.setOrigin(tf2::Vector3(
            msg->pose.pose.position.x,
            msg->pose.pose.position.y,
            msg->pose.pose.position.z));
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        q.normalize();
        T_map_base.setRotation(q);

        double x = 0.0, y = 0.0, z = 0.0;
        double roll = 0.0, pitch = 0.0, yaw = 0.0;
        {
            std::lock_guard<std::mutex> lock(tf_mutex_);
            if (!have_tf_B_out_) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                     "尚未收到 odom->base_link，忽略 /initialpose。");
                return;
            }
            tf_A_ = T_map_base * tf_B_out_.inverse();
            const tf2::Vector3 t = tf_A_.getOrigin();
            x = t.x(); y = t.y(); z = t.z();
            tf2::Matrix3x3(tf_A_.getRotation()).getRPY(roll, pitch, yaw);
            map_to_odom_x_ = x;
            map_to_odom_y_ = y;
            map_to_odom_z_ = z;
            map_to_odom_roll_ = roll;
            map_to_odom_pitch_ = pitch;
            map_to_odom_yaw_ = yaw;
        }
        RCLCPP_INFO(this->get_logger(),
                    "收到 /initialpose，更新 map->odom: x=%.3f y=%.3f z=%.3f "
                    "roll=%.2f pitch=%.2f yaw=%.2f deg",
                    x, y, z,
                    roll * 180.0 / M_PI,
                    pitch * 180.0 / M_PI,
                    yaw * 180.0 / M_PI);
    }

    static double yawFromQuaternion(const tf2::Quaternion &q)
    {
        return std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                          1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
    }

    void lioOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        if (!msg) return;

        tf2::Transform sensor_pose;
        tf2::fromMsg(msg->pose.pose, sensor_pose);
        // 先把参考点移到底盘原点，避免把雷达绕底盘转动的切向速度计入 /odom。
        const OdomSample sample{rclcpp::Time(msg->header.stamp), sensor_pose * tf_C_inv_};

        std::lock_guard<std::mutex> lock(odom_mutex_);
        last_lio_odom_ = *msg;

        odom_history_.push_back(sample);
        while (!odom_history_.empty() &&
               (sample.stamp - odom_history_.front().stamp).seconds() > 0.5) {
            odom_history_.pop_front();
        }

        // 用约 0.1s 前的位姿做差分，降低单帧噪声。
        constexpr double kVelocityWindow = 0.10;
        const OdomSample *ref = &odom_history_.front();
        for (auto it = odom_history_.rbegin(); it != odom_history_.rend(); ++it) {
            if ((sample.stamp - it->stamp).seconds() >= kVelocityWindow) {
                ref = &(*it);
                break;
            }
        }

        const double dt = (sample.stamp - ref->stamp).seconds();
        if (dt > 0.02) {
            const tf2::Vector3 v_odom =
                (sample.base_pose.getOrigin() - ref->base_pose.getOrigin()) / dt;
            body_linear_vel_ = sample.base_pose.getBasis().transpose() * v_odom;

            const auto relative_rotation =
                ref->base_pose.getRotation().inverse() * sample.base_pose.getRotation();
            const double dyaw = yawFromQuaternion(relative_rotation);
            body_angular_vel_ = tf2::Vector3(0.0, 0.0, dyaw / dt);
        } else {
            body_linear_vel_.setZero();
            body_angular_vel_.setZero();
        }
    }

    void timerCallback()
    {
        rclcpp::Time now = this->now();

        nav_msgs::msg::Odometry lio;
        tf2::Transform tf_B;
        tf2::Vector3 body_linear_vel(0.0, 0.0, 0.0);
        tf2::Vector3 body_angular_vel(0.0, 0.0, 0.0);
        {
            std::lock_guard<std::mutex> lock(odom_mutex_);
            if (odom_history_.empty()) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                     "等待 /lio/robo/odom（nav_msgs/Odometry）...");
                return;
            }
            lio = last_lio_odom_;
            tf_B = odom_history_.back().base_pose;
            body_linear_vel = body_linear_vel_;
            body_angular_vel = body_angular_vel_;
        }

        const double age = (now - lio.header.stamp).seconds();
        if (age > 5.0) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "/lio/robo/odom 过旧(%.2fs)，暂不更新 B", age);
            return;
        }

        try {
            // 可选：把首帧 B 作为零点，使启动时 base_link 与 map/odom 对齐。
            tf2::Transform tf_B_out = tf_B;
            if (align_base_link_to_map_on_start_) {
                if (!have_initial_tf_B_) {
                    tf_B0_inv_ = tf_B.inverse();
                    have_initial_tf_B_ = true;
                    RCLCPP_INFO(this->get_logger(),
                                "已记录首帧 odom->base_link，后续发布相对位姿（首帧归零）。");
                }
                tf_B_out = tf_B0_inv_ * tf_B;
            }

            // 保存最新 odom->base_link，供 /initialpose 反算 map->odom
            {
                std::lock_guard<std::mutex> lock(tf_mutex_);
                tf_B_out_ = tf_B_out;
                have_tf_B_out_ = true;
            }

            // 发布 B(odom->base_link) TF
            geometry_msgs::msg::TransformStamped B_msg;
            B_msg.header.stamp = lio.header.stamp;
            B_msg.header.frame_id = odom_frame_;
            B_msg.child_frame_id = "base_link";
            B_msg.transform = tf2::toMsg(tf_B_out);
            tf_broadcaster_->sendTransform(B_msg);

            // 发布 A(map->odom) 动态 TF（/initialpose 可实时更新）
            if (publish_map_to_odom_) {
                tf2::Transform tf_A_copy;
                {
                    std::lock_guard<std::mutex> lock(tf_mutex_);
                    tf_A_copy = tf_A_;
                }
                geometry_msgs::msg::TransformStamped A_msg;
                A_msg.header.stamp = B_msg.header.stamp;
                A_msg.header.frame_id = map_frame_for_A_;
                A_msg.child_frame_id = odom_frame_;
                A_msg.transform = tf2::toMsg(tf_A_copy);
                tf_broadcaster_->sendTransform(A_msg);
            }

            // 发布 /odom（保持原来速度/协方差映射）
            nav_msgs::msg::Odometry odom_msg;
            odom_msg.header.stamp = B_msg.header.stamp;
            odom_msg.header.frame_id = odom_frame_;
            odom_msg.child_frame_id = "base_link";

            odom_msg.pose.pose.position.x = tf_B_out.getOrigin().x();
            odom_msg.pose.pose.position.y = tf_B_out.getOrigin().y();
            odom_msg.pose.pose.position.z = tf_B_out.getOrigin().z();
            odom_msg.pose.pose.orientation = tf2::toMsg(tf_B_out.getRotation());

            // 位姿差分得到的 base_link 系速度（Super-LIO 原始 twist 恒为 0）
            odom_msg.twist.twist.linear.x = body_linear_vel.x();
            odom_msg.twist.twist.linear.y = body_linear_vel.y();
            odom_msg.twist.twist.linear.z = body_linear_vel.z();
            odom_msg.twist.twist.angular.x = body_angular_vel.x();
            odom_msg.twist.twist.angular.y = body_angular_vel.y();
            odom_msg.twist.twist.angular.z = body_angular_vel.z();

            odom_msg.pose.covariance = lio.pose.covariance;
            odom_msg.twist.covariance = lio.twist.covariance;

            odom_publisher_->publish(odom_msg);

        } catch (const std::exception &e) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "计算/发布 B 失败: %s", e.what());
        }
    }

private:
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr lio_odom_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
        initial_pose_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;

    // C: base_link -> livox_frame
    tf2::Transform tf_C_;
    tf2::Transform tf_C_inv_;
    double livox_offset_x_ = 0.0;
    double livox_offset_y_ = 0.0;
    double livox_offset_z_ = 0.0;
    double livox_offset_roll_ = 0.0;
    double livox_offset_pitch_ = 0.0;
    double livox_offset_yaw_ = 0.0;

    // A: map -> odom
    tf2::Transform tf_A_;
    double map_to_odom_x_ = 0.0;
    double map_to_odom_y_ = 0.0;
    double map_to_odom_z_ = 0.0;
    double map_to_odom_roll_ = 0.0;
    double map_to_odom_pitch_ = 0.0;
    double map_to_odom_yaw_ = 0.0;

    tf2::Transform tf_B_out_;
    bool have_tf_B_out_ = false;
    std::mutex tf_mutex_;

    // E 来源为 /lio/robo/odom
    struct OdomSample
    {
        rclcpp::Time stamp;
        tf2::Transform base_pose;
    };

    nav_msgs::msg::Odometry last_lio_odom_;
    std::deque<OdomSample> odom_history_;
    std::mutex odom_mutex_;
    tf2::Vector3 body_linear_vel_{0.0, 0.0, 0.0};
    tf2::Vector3 body_angular_vel_{0.0, 0.0, 0.0};

    bool align_base_link_to_map_on_start_ = true;
    bool publish_map_to_odom_ = true;
    bool have_initial_tf_B_ = false;
    tf2::Transform tf_B0_inv_;

    std::string map_frame_for_A_ = "map";
    std::string odom_frame_ = "odom";
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<TfOnlyOdom>();
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();

    rclcpp::shutdown();
    return 0;
}
