// Copyright 2026 QDU RoboMaster Team
//
// Licensed under the Apache License, Version 2.0

#include <algorithm>
#include <cmath>
#include <cfloat>
#include <deque>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/common/transforms.h"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_eigen/tf2_eigen.hpp"

#include "Eigen/Geometry"

#pragma pack(push, 1)

struct PointMid360
{
  float x;
  float y;
  float z;
  float intensity;
  uint8_t tag;
  uint8_t line;
  double timestamp;
};

struct PointOdin1
{
  float x;
  float y;
  float z;
  uint8_t intensity;
  uint16_t confidence;
  float offset_time;
};

#pragma pack(pop)

POINT_CLOUD_REGISTER_POINT_STRUCT(PointMid360,
  (float, x, x)
  (float, y, y)
  (float, z, z)
  (float, intensity, intensity)
  (uint8_t, tag, tag)
  (uint8_t, line, line)
  (double, timestamp, timestamp)
)

POINT_CLOUD_REGISTER_POINT_STRUCT(PointOdin1,
  (float, x, x)
  (float, y, y)
  (float, z, z)
  (uint8_t, intensity, intensity)
  (uint16_t, confidence, confidence)
  (float, offset_time, offset_time)
)

class PointCloudFusionNode : public rclcpp::Node
{
public:
  PointCloudFusionNode()
  : Node("pointcloud_fusion")
  {
    this->declare_parameter("odin1_topic", "odin1/cloud_raw");
    this->declare_parameter("mid360_topic", "/livox/lidar");
    this->declare_parameter("output_topic", "fused_pointcloud");

    // 建议先输出到 base_footprint，避免输出到 odom 时强依赖动态 TF。
    this->declare_parameter("output_frame", "base_footprint");
    this->declare_parameter("robot_frame", "base_footprint");

    // 先关闭 deskew，保证比赛场景实时融合稳定。
    // 后续 TF 稳定后可以改 true。
    this->declare_parameter("enable_deskew", true);

    // TF 查询指定时间失败时，退化使用最新 TF，避免整帧丢弃。
    this->declare_parameter("allow_latest_tf_fallback", true);

    // 如果点云时间比 TF 最新时间更靠前，仍使用最新 TF，不丢点云。
    this->declare_parameter("tf_future_tolerance_sec", 0.30);

    // Mid-360 时间戳相对主机时间的偏移（秒）。mid360_stamp += offset。
    // mid360 用传感器内部时钟时与 odin1（主机时间）有固定偏差，用此参数校正。
    this->declare_parameter("mid360_stamp_offset_sec", 0.0);

    // Odin1 和 Mid-360 最大允许匹配时间差。
    // Mid-360 10 Hz，一帧周期 0.1s，建议 0.10~0.15。
    this->declare_parameter("max_sync_diff_sec", 0.15);

    // 缓存最近几帧 Mid-360。
    this->declare_parameter("mid360_queue_size", 5);

    // 没匹配到 Mid-360 时是否仍发布 Odin1 点云。
    this->declare_parameter("publish_without_mid360", true);

    // 输出 QoS。true 兼容 Nav2 / slam_toolbox / 很多默认 Reliable 订阅。
    this->declare_parameter("publish_reliable", true);

    this->declare_parameter("self_filter_half_x", 0.3);
    this->declare_parameter("self_filter_half_y", 0.3);
    this->declare_parameter("self_filter_height", 0.6);
    this->declare_parameter("self_filter_min_z", -0.5);

    const std::string odin1_topic = this->get_parameter("odin1_topic").as_string();
    const std::string mid360_topic = this->get_parameter("mid360_topic").as_string();
    const std::string output_topic = this->get_parameter("output_topic").as_string();

    output_frame_ = this->get_parameter("output_frame").as_string();
    robot_frame_ = this->get_parameter("robot_frame").as_string();

    enable_deskew_ = this->get_parameter("enable_deskew").as_bool();
    allow_latest_tf_fallback_ = this->get_parameter("allow_latest_tf_fallback").as_bool();
    tf_future_tolerance_sec_ = this->get_parameter("tf_future_tolerance_sec").as_double();

    mid360_stamp_offset_sec_ = this->get_parameter("mid360_stamp_offset_sec").as_double();
    max_sync_diff_sec_ = this->get_parameter("max_sync_diff_sec").as_double();
    mid360_queue_size_ = static_cast<size_t>(
      std::max<int64_t>(1, this->get_parameter("mid360_queue_size").as_int()));
    publish_without_mid360_ = this->get_parameter("publish_without_mid360").as_bool();

    publish_reliable_ = this->get_parameter("publish_reliable").as_bool();

    self_filter_half_x_ = this->get_parameter("self_filter_half_x").as_double();
    self_filter_half_y_ = this->get_parameter("self_filter_half_y").as_double();
    self_filter_height_ = this->get_parameter("self_filter_height").as_double();
    self_filter_min_z_ = this->get_parameter("self_filter_min_z").as_double();

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    rmw_qos_profile_t sensor_rmw_qos = rmw_qos_profile_sensor_data;
    sensor_qos_ = rclcpp::QoS(
      rclcpp::QoSInitialization::from_rmw(sensor_rmw_qos),
      sensor_rmw_qos);

    // Mid-360 使用 RELIABLE QoS 匹配 livox 驱动，避免消息丢失。
    rclcpp::QoS mid360_qos(rclcpp::KeepLast(5));
    mid360_qos.reliable();
    mid360_qos.durability_volatile();

    odin_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    mid_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions odin_options;
    odin_options.callback_group = odin_group_;

    rclcpp::SubscriptionOptions mid_options;
    mid_options.callback_group = mid_group_;

    odin1_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      odin1_topic,
      sensor_qos_,
      std::bind(&PointCloudFusionNode::odin1Callback, this, std::placeholders::_1),
      odin_options);

    mid360_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      mid360_topic,
      mid360_qos,
      std::bind(&PointCloudFusionNode::mid360Callback, this, std::placeholders::_1),
      mid_options);

    rclcpp::QoS pub_qos(rclcpp::KeepLast(1));
    pub_qos.durability_volatile();

    if (publish_reliable_) {
      pub_qos.reliable();
    } else {
      pub_qos.best_effort();
    }

    fused_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      output_topic,
      pub_qos);

    RCLCPP_INFO(
      this->get_logger(),
      "PointCloud Fusion started.");
    RCLCPP_INFO(
      this->get_logger(),
      "Odin1 topic: %s", odin1_topic.c_str());
    RCLCPP_INFO(
      this->get_logger(),
      "Mid-360 topic: %s", mid360_topic.c_str());
    RCLCPP_INFO(
      this->get_logger(),
      "Output topic: %s", output_topic.c_str());
    RCLCPP_INFO(
      this->get_logger(),
      "output_frame=%s, robot_frame=%s, enable_deskew=%s, max_sync_diff=%.3f",
      output_frame_.c_str(),
      robot_frame_.c_str(),
      enable_deskew_ ? "true" : "false",
      max_sync_diff_sec_);
  }

private:
  struct CachedCloud
  {
    double stamp_sec;
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> cloud;
  };

  static double stampToSec(const builtin_interfaces::msg::Time & stamp)
  {
    return static_cast<double>(stamp.sec) +
           static_cast<double>(stamp.nanosec) * 1e-9;
  }

  static rclcpp::Time secToTime(double sec)
  {
    if (!std::isfinite(sec) || sec <= 0.0) {
      return rclcpp::Time(0, 0, RCL_ROS_TIME);
    }

    const int64_t ns = static_cast<int64_t>(sec * 1e9);
    return rclcpp::Time(ns, RCL_ROS_TIME);
  }

  static bool isValidPoint(float x, float y, float z)
  {
    return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
  }

  static geometry_msgs::msg::TransformStamped makeIdentityTransform(
    const std::string & target_frame,
    const std::string & source_frame,
    double stamp_sec)
  {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.frame_id = target_frame;
    tf.child_frame_id = source_frame;
    tf.header.stamp = secToTime(stamp_sec);

    tf.transform.translation.x = 0.0;
    tf.transform.translation.y = 0.0;
    tf.transform.translation.z = 0.0;

    tf.transform.rotation.x = 0.0;
    tf.transform.rotation.y = 0.0;
    tf.transform.rotation.z = 0.0;
    tf.transform.rotation.w = 1.0;

    return tf;
  }

  double convertMid360RawTimestampToSec(double raw, double header_sec) const
  {
    if (!std::isfinite(raw) || raw <= 0.0) {
      return std::numeric_limits<double>::quiet_NaN();
    }

    // Unix ns，例如 1778177512212934000
    if (raw > 1.0e14) {
      return raw * 1e-9;
    }

    // Unix 秒，例如 1778177512.212934
    if (raw > 1.0e9) {
      if (header_sec > 1.0e6 && std::fabs(raw - header_sec) < 10.0) {
        return raw;
      }

      // 更可能是 ns 级相对时间。
      return raw * 1e-9;
    }

    // ns 级相对时间。
    if (raw > 1.0e6) {
      return raw * 1e-9;
    }

    // 秒级相对时间。
    return raw;
  }

  bool buildMid360Times(
    const pcl::PointCloud<PointMid360> & raw,
    double header_sec,
    std::vector<double> & point_times,
    double & t_min,
    double & t_max,
    double & frame_stamp_sec)
  {
    point_times.clear();
    point_times.reserve(raw.size());

    std::vector<double> candidate_times;
    candidate_times.reserve(raw.size());

    double c_min = DBL_MAX;
    double c_max = -DBL_MAX;
    size_t valid_time_count = 0;

    for (const auto & p : raw) {
      const double t = convertMid360RawTimestampToSec(p.timestamp, header_sec);
      candidate_times.push_back(t);

      if (std::isfinite(t) && t > 0.0) {
        c_min = std::min(c_min, t);
        c_max = std::max(c_max, t);
        ++valid_time_count;
      }
    }

    if (valid_time_count == 0 || c_max < c_min) {
      point_times.assign(raw.size(), header_sec);
      t_min = header_sec;
      t_max = header_sec;
      frame_stamp_sec = header_sec;
      return false;
    }

    const double c_mid = 0.5 * (c_min + c_max);
    const double duration = c_max - c_min;

    const bool header_looks_like_unix = header_sec > 1.0e6;
    const bool candidate_matches_header =
      header_looks_like_unix && std::fabs(c_mid - header_sec) < 5.0;

    if (candidate_matches_header) {
      point_times = candidate_times;
      t_min = c_min;
      t_max = c_max;
      frame_stamp_sec = c_mid;
      return true;
    }

    if (header_looks_like_unix && duration >= 0.0 && duration < 0.3) {
      point_times.resize(candidate_times.size(), header_sec);

      t_min = DBL_MAX;
      t_max = -DBL_MAX;

      for (size_t i = 0; i < candidate_times.size(); ++i) {
        double shifted = header_sec;

        if (std::isfinite(candidate_times[i])) {
          shifted = header_sec + (candidate_times[i] - c_mid);
        }

        point_times[i] = shifted;
        t_min = std::min(t_min, shifted);
        t_max = std::max(t_max, shifted);
      }

      frame_stamp_sec = header_sec;
      return true;
    }

    point_times.assign(raw.size(), header_sec);
    t_min = header_sec;
    t_max = header_sec;
    frame_stamp_sec = header_sec;

    RCLCPP_DEBUG_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      1000,
      "Mid-360 point timestamp does not match header. Use header stamp only. "
      "header=%.6f, point_mid=%.6f, duration=%.6f",
      header_sec,
      c_mid,
      duration);

    return false;
  }

  bool lookupTransformWithFallback(
    geometry_msgs::msg::TransformStamped & tf,
    const std::string & target_frame,
    const std::string & source_frame,
    double stamp_sec,
    const char * log_prefix)
  {
    if (target_frame == source_frame) {
      tf = makeIdentityTransform(target_frame, source_frame, stamp_sec);
      return true;
    }

    try {
      tf = tf_buffer_->lookupTransform(
        target_frame,
        source_frame,
        secToTime(stamp_sec));
      return true;
    } catch (const tf2::TransformException & ex) {
      if (!allow_latest_tf_fallback_) {
        RCLCPP_DEBUG_THROTTLE(
          this->get_logger(),
          *this->get_clock(),
          1000,
          "%s TF at stamp failed: %s",
          log_prefix,
          ex.what());
        return false;
      }

      try {
        tf = tf_buffer_->lookupTransform(
          target_frame,
          source_frame,
          tf2::TimePointZero);

        RCLCPP_DEBUG_THROTTLE(
          this->get_logger(),
          *this->get_clock(),
          1000,
          "%s TF at stamp failed, fallback to latest TF: %s",
          log_prefix,
          ex.what());

        return true;
      } catch (const tf2::TransformException & ex2) {
        RCLCPP_DEBUG_THROTTLE(
          this->get_logger(),
          *this->get_clock(),
          1000,
          "%s latest TF also failed: %s",
          log_prefix,
          ex2.what());
        return false;
      }
    }
  }

  bool transformWholeCloud(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud,
    const std::string & source_frame,
    double stamp_sec,
    const char * log_prefix)
  {
    if (source_frame.empty()) {
      RCLCPP_DEBUG_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000,
        "%s source frame is empty.",
        log_prefix);
      return false;
    }

    if (source_frame == output_frame_) {
      return true;
    }

    geometry_msgs::msg::TransformStamped tf;

    if (!lookupTransformWithFallback(
        tf,
        output_frame_,
        source_frame,
        stamp_sec,
        log_prefix))
    {
      return false;
    }

    const Eigen::Affine3d eigen_tf = tf2::transformToEigen(tf.transform);
    pcl::transformPointCloud(*cloud, *cloud, eigen_tf);
    return true;
  }

  bool deskewAndTransform(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud,
    const std::vector<double> & point_times,
    const std::string & source_frame,
    double t_min,
    double t_max,
    double fallback_stamp_sec,
    const char * log_prefix)
  {
    if (source_frame.empty()) {
      RCLCPP_DEBUG_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000,
        "%s source frame is empty.",
        log_prefix);
      return false;
    }

    if (source_frame == output_frame_) {
      return true;
    }

    if (cloud->empty()) {
      return true;
    }

    if (point_times.size() != cloud->size()) {
      RCLCPP_DEBUG_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000,
        "%s point_times size mismatch. Use whole-frame transform.",
        log_prefix);

      return transformWholeCloud(cloud, source_frame, fallback_stamp_sec, log_prefix);
    }

    if (!std::isfinite(t_min) || !std::isfinite(t_max) ||
        t_min <= 0.0 || t_max <= 0.0 ||
        t_max - t_min < 1e-6)
    {
      return transformWholeCloud(cloud, source_frame, fallback_stamp_sec, log_prefix);
    }

    try {
      const auto latest_tf = tf_buffer_->lookupTransform(
        output_frame_,
        source_frame,
        tf2::TimePointZero);

      const double t_latest = stampToSec(latest_tf.header.stamp);

      // 静态 TF 的 stamp 可能是 0，直接整帧变换。
      if (t_latest <= 1e-6) {
        const Eigen::Affine3d eigen_tf = tf2::transformToEigen(latest_tf.transform);
        pcl::transformPointCloud(*cloud, *cloud, eigen_tf);
        return true;
      }

      // 重点修复：
      // 点云时间比 TF 最新时间更靠前时，不再丢帧。
      // 直接用最新 TF 退化整帧变换，避免 Mid-360 缓存停住。
      if (t_min >= t_latest - 1e-4) {
        const double ahead = t_min - t_latest;

        const Eigen::Affine3d eigen_tf = tf2::transformToEigen(latest_tf.transform);
        pcl::transformPointCloud(*cloud, *cloud, eigen_tf);

        if (ahead > tf_future_tolerance_sec_) {
          RCLCPP_DEBUG_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "%s cloud is ahead of TF by %.3f s. Still use latest TF to avoid dropping cloud.",
            log_prefix,
            ahead);
        } else {
          RCLCPP_DEBUG_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "%s cloud is slightly ahead of TF by %.3f s. Use latest TF.",
            log_prefix,
            ahead);
        }

        return true;
      }

      if (t_max > t_latest) {
        t_max = t_latest;
      }

      const auto tf_start = tf_buffer_->lookupTransform(
        output_frame_,
        source_frame,
        secToTime(t_min));

      const auto tf_end = tf_buffer_->lookupTransform(
        output_frame_,
        source_frame,
        secToTime(t_max));

      const Eigen::Affine3d aff_start = tf2::transformToEigen(tf_start.transform);
      const Eigen::Affine3d aff_end = tf2::transformToEigen(tf_end.transform);

      const Eigen::Quaterniond q_start(aff_start.rotation());
      const Eigen::Quaterniond q_end(aff_end.rotation());

      const Eigen::Vector3d tr_start = aff_start.translation();
      const Eigen::Vector3d tr_end = aff_end.translation();

      const double inv_dt = 1.0 / std::max(1e-9, t_max - t_min);

      for (size_t i = 0; i < cloud->size(); ++i) {
        const double alpha = std::clamp(
          (point_times[i] - t_min) * inv_dt,
          0.0,
          1.0);

        const Eigen::Quaterniond q_interp = q_start.slerp(alpha, q_end);
        const Eigen::Vector3d tr_interp = tr_start + alpha * (tr_end - tr_start);

        const Eigen::Vector3d pt(
          cloud->points[i].x,
          cloud->points[i].y,
          cloud->points[i].z);

        const Eigen::Vector3d pt_out = q_interp * pt + tr_interp;

        cloud->points[i].x = static_cast<float>(pt_out.x());
        cloud->points[i].y = static_cast<float>(pt_out.y());
        cloud->points[i].z = static_cast<float>(pt_out.z());
      }

      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_DEBUG_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000,
        "%s deskew TF failed, fallback to whole-frame transform: %s",
        log_prefix,
        ex.what());

      return transformWholeCloud(
        cloud,
        source_frame,
        fallback_stamp_sec,
        log_prefix);
    }
  }

  void mid360Callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    pcl::PointCloud<PointMid360>::Ptr raw(new pcl::PointCloud<PointMid360>);
    pcl::fromROSMsg(*msg, *raw);

    if (raw->empty()) {
      RCLCPP_DEBUG_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000,
        "Mid-360 cloud is empty.");
      return;
    }

    const double header_sec = stampToSec(msg->header.stamp);

    std::vector<double> point_times;
    double t_min = header_sec;
    double t_max = header_sec;
    double frame_stamp_sec = header_sec;

    buildMid360Times(
      *raw,
      header_sec,
      point_times,
      t_min,
      t_max,
      frame_stamp_sec);

    auto xyzi = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    xyzi->reserve(raw->size());

    std::vector<double> valid_point_times;
    valid_point_times.reserve(raw->size());

    for (size_t i = 0; i < raw->size(); ++i) {
      const auto & p = raw->points[i];

      if (!isValidPoint(p.x, p.y, p.z)) {
        continue;
      }

      pcl::PointXYZI pi;
      pi.x = p.x;
      pi.y = p.y;
      pi.z = p.z;
      pi.intensity = p.intensity;

      xyzi->push_back(pi);

      if (i < point_times.size()) {
        valid_point_times.push_back(point_times[i]);
      } else {
        valid_point_times.push_back(frame_stamp_sec);
      }
    }

    if (xyzi->empty()) {
      return;
    }

    if (valid_point_times.size() != xyzi->size()) {
      valid_point_times.assign(xyzi->size(), frame_stamp_sec);
      t_min = frame_stamp_sec;
      t_max = frame_stamp_sec;
    }

    bool ok = false;

    if (enable_deskew_) {
      ok = deskewAndTransform(
        xyzi,
        valid_point_times,
        msg->header.frame_id,
        t_min,
        t_max,
        frame_stamp_sec,
        "Mid-360");
    } else {
      ok = transformWholeCloud(
        xyzi,
        msg->header.frame_id,
        frame_stamp_sec,
        "Mid-360");
    }

    if (!ok) {
      RCLCPP_DEBUG_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000,
        "Drop Mid-360 frame because transform failed. frame=%s, stamp=%.6f",
        msg->header.frame_id.c_str(),
        frame_stamp_sec);
      return;
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);

      mid360_queue_.push_back(CachedCloud{frame_stamp_sec + mid360_stamp_offset_sec_, xyzi});

      while (mid360_queue_.size() > mid360_queue_size_) {
        mid360_queue_.pop_front();
      }
    }

    RCLCPP_DEBUG(
      this->get_logger(),
      "Cached Mid-360 cloud. stamp=%.6f, points=%zu",
      frame_stamp_sec,
      xyzi->size());
  }

  bool findClosestMid360(
    double odin_stamp_sec,
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> & mid_cloud,
    double & mid_stamp_sec,
    double & best_dt)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (mid360_queue_.empty()) {
      mid_cloud.reset();
      mid_stamp_sec = 0.0;
      best_dt = std::numeric_limits<double>::infinity();
      return false;
    }

    auto best_it = mid360_queue_.begin();
    best_dt = std::fabs(odin_stamp_sec - best_it->stamp_sec);

    for (auto it = mid360_queue_.begin(); it != mid360_queue_.end(); ++it) {
      const double dt = std::fabs(odin_stamp_sec - it->stamp_sec);

      if (dt < best_dt) {
        best_dt = dt;
        best_it = it;
      }
    }

    mid_cloud = best_it->cloud;
    mid_stamp_sec = best_it->stamp_sec;

    return best_dt <= max_sync_diff_sec_;
  }

  void buildOdin1CloudAndTimes(
    const pcl::PointCloud<PointOdin1> & raw,
    double header_sec,
    pcl::PointCloud<pcl::PointXYZI>::Ptr & xyzi,
    std::vector<double> & point_times,
    double & t_min,
    double & t_max)
  {
    xyzi = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    xyzi->reserve(raw.size());

    point_times.clear();
    point_times.reserve(raw.size());

    t_min = DBL_MAX;
    t_max = -DBL_MAX;

    for (const auto & p : raw) {
      if (!isValidPoint(p.x, p.y, p.z)) {
        continue;
      }

      pcl::PointXYZI pi;
      pi.x = p.x;
      pi.y = p.y;
      pi.z = p.z;
      pi.intensity = static_cast<float>(p.intensity);

      xyzi->push_back(pi);

      double point_time = header_sec;

      if (std::isfinite(p.offset_time)) {
        point_time = header_sec + static_cast<double>(p.offset_time);
      }

      point_times.push_back(point_time);
      t_min = std::min(t_min, point_time);
      t_max = std::max(t_max, point_time);
    }

    if (xyzi->empty()) {
      t_min = header_sec;
      t_max = header_sec;
      return;
    }

    if (!std::isfinite(t_min) || !std::isfinite(t_max) || t_max < t_min) {
      point_times.assign(xyzi->size(), header_sec);
      t_min = header_sec;
      t_max = header_sec;
    }
  }

  void applySelfFilter(
    pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud,
    double stamp_sec)
  {
    if (!cloud || cloud->empty()) {
      return;
    }

    geometry_msgs::msg::TransformStamped tf;

    if (!lookupTransformWithFallback(
        tf,
        output_frame_,
        robot_frame_,
        stamp_sec,
        "Self-filter"))
    {
      return;
    }

    const Eigen::Affine3d robot_to_output = tf2::transformToEigen(tf.transform);
    const Eigen::Affine3d output_to_robot = robot_to_output.inverse();

    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZI>);
    filtered->reserve(cloud->size());

    for (const auto & p : *cloud) {
      const Eigen::Vector3d pt_output(p.x, p.y, p.z);
      const Eigen::Vector3d pt_robot = output_to_robot * pt_output;

      const double dx = pt_robot.x();
      const double dy = pt_robot.y();
      const double dz = pt_robot.z();

      if (dx > -self_filter_half_x_ && dx < self_filter_half_x_ &&
          dy > -self_filter_half_y_ && dy < self_filter_half_y_ &&
          dz > self_filter_min_z_ && dz < self_filter_height_)
      {
        continue;
      }

      filtered->push_back(p);
    }

    cloud = filtered;
  }

  void odin1Callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    pcl::PointCloud<PointOdin1>::Ptr raw(new pcl::PointCloud<PointOdin1>);
    pcl::fromROSMsg(*msg, *raw);

    if (raw->empty()) {
      RCLCPP_DEBUG_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000,
        "Odin1 cloud is empty.");
      return;
    }

    const double odin_stamp_sec = stampToSec(msg->header.stamp);

    pcl::PointCloud<pcl::PointXYZI>::Ptr odin_xyzi;
    std::vector<double> odin_times;
    double odin_t_min = odin_stamp_sec;
    double odin_t_max = odin_stamp_sec;

    buildOdin1CloudAndTimes(
      *raw,
      odin_stamp_sec,
      odin_xyzi,
      odin_times,
      odin_t_min,
      odin_t_max);

    if (!odin_xyzi || odin_xyzi->empty()) {
      return;
    }

    bool odin_ok = false;

    if (enable_deskew_) {
      odin_ok = deskewAndTransform(
        odin_xyzi,
        odin_times,
        msg->header.frame_id,
        odin_t_min,
        odin_t_max,
        odin_stamp_sec,
        "Odin1");
    } else {
      odin_ok = transformWholeCloud(
        odin_xyzi,
        msg->header.frame_id,
        odin_stamp_sec,
        "Odin1");
    }

    if (!odin_ok) {
      RCLCPP_DEBUG_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000,
        "Drop Odin1 frame because transform failed. frame=%s, stamp=%.6f",
        msg->header.frame_id.c_str(),
        odin_stamp_sec);
      return;
    }

    std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> mid_cloud;
    double mid_stamp_sec = 0.0;
    double mid_dt = std::numeric_limits<double>::infinity();

    const bool has_synced_mid = findClosestMid360(
      odin_stamp_sec,
      mid_cloud,
      mid_stamp_sec,
      mid_dt);

    if (!has_synced_mid) {
      if (mid_cloud) {
        RCLCPP_DEBUG_THROTTLE(
          this->get_logger(),
          *this->get_clock(),
          1000,
          "No synced Mid-360 frame. Closest dt=%.3f s, odin=%.6f, mid=%.6f. "
          "Skip Mid-360 for this output.",
          mid_dt,
          odin_stamp_sec,
          mid_stamp_sec);
      } else {
        RCLCPP_DEBUG_THROTTLE(
          this->get_logger(),
          *this->get_clock(),
          1000,
          "No Mid-360 frame cached yet.");
      }

      if (!publish_without_mid360_) {
        return;
      }

      mid_cloud.reset();
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr fused(new pcl::PointCloud<pcl::PointXYZI>);

    const size_t mid_size = mid_cloud ? mid_cloud->size() : 0;
    fused->reserve(odin_xyzi->size() + mid_size);

    *fused += *odin_xyzi;

    if (mid_cloud) {
      *fused += *mid_cloud;
    }

    applySelfFilter(fused, odin_stamp_sec);

    sensor_msgs::msg::PointCloud2 out;
    pcl::toROSMsg(*fused, out);

    out.header.stamp = msg->header.stamp;
    out.header.frame_id = output_frame_;

    fused_pub_->publish(out);

    RCLCPP_DEBUG(
      this->get_logger(),
      "Publish fused cloud. odin_points=%zu, mid_points=%zu, fused_points=%zu, mid_dt=%.3f",
      odin_xyzi->size(),
      mid_size,
      fused->size(),
      mid_dt);
  }

private:
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::CallbackGroup::SharedPtr odin_group_;
  rclcpp::CallbackGroup::SharedPtr mid_group_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr odin1_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr mid360_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr fused_pub_;

  rclcpp::QoS sensor_qos_{rclcpp::KeepLast(5)};

  std::deque<CachedCloud> mid360_queue_;
  std::mutex mutex_;

  std::string output_frame_;
  std::string robot_frame_;

  bool enable_deskew_{false};
  bool allow_latest_tf_fallback_{true};
  bool publish_without_mid360_{true};
  bool publish_reliable_{true};

  double max_sync_diff_sec_{0.15};
  double tf_future_tolerance_sec_{0.30};
  double mid360_stamp_offset_sec_{0.0};

  size_t mid360_queue_size_{5};

  double self_filter_half_x_{0.3};
  double self_filter_half_y_{0.3};
  double self_filter_height_{0.6};
  double self_filter_min_z_{-0.5};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<PointCloudFusionNode>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
