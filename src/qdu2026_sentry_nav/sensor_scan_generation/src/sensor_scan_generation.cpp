// Copyright 2025 Lihan Chen
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "sensor_scan_generation/sensor_scan_generation.hpp"

#include "pcl_ros/transforms.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace sensor_scan_generation
{

SensorScanGenerationNode::SensorScanGenerationNode(const rclcpp::NodeOptions & options)
: Node("sensor_scan_generation", options)
{
  this->declare_parameter<std::string>("lidar_frame", "");
  this->declare_parameter<std::string>("base_frame", "");
  this->declare_parameter<std::string>("robot_base_frame", "");
  this->declare_parameter<double>("yaw_compensation_factor", 0.0);
  this->declare_parameter<double>("velocity_filter_alpha", 0.2);

  this->get_parameter("lidar_frame", lidar_frame_);
  this->get_parameter("base_frame", base_frame_);
  this->get_parameter("robot_base_frame", robot_base_frame_);
  this->get_parameter("yaw_compensation_factor", yaw_compensation_factor_);
  this->get_parameter("velocity_filter_alpha", velocity_filter_alpha_);

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  br_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  pub_laser_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("sensor_scan", 2);
  pub_chassis_odometry_ = this->create_publisher<nav_msgs::msg::Odometry>("odometry", 2);

  rmw_qos_profile_t qos_profile = {
    RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    1,
    RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
    RMW_QOS_POLICY_DURABILITY_VOLATILE,
    RMW_QOS_DEADLINE_DEFAULT,
    RMW_QOS_LIFESPAN_DEFAULT,
    RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
    RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
    false};

  auto sensor_qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(qos_profile), qos_profile);
  direct_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "lidar_odometry", sensor_qos,
    std::bind(&SensorScanGenerationNode::odometryHandler, this, std::placeholders::_1));

  odometry_sub_.subscribe(this, "lidar_odometry", qos_profile);
  laser_cloud_sub_.subscribe(this, "registered_scan", qos_profile);

  sync_ = std::make_unique<message_filters::Synchronizer<SyncPolicy>>(
    SyncPolicy(100), odometry_sub_, laser_cloud_sub_);
  sync_->registerCallback(std::bind(
    &SensorScanGenerationNode::laserCloudAndOdometryHandler, this, std::placeholders::_1,
    std::placeholders::_2));

  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "/livox/imu", rclcpp::SensorDataQoS(),
    std::bind(&SensorScanGenerationNode::imuCallback, this, std::placeholders::_1));
}

void SensorScanGenerationNode::imuCallback(
  const sensor_msgs::msg::Imu::ConstSharedPtr & msg)
{
  double raw_yaw_velocity = msg->angular_velocity.z;

  filtered_yaw_velocity_ =
    velocity_filter_alpha_ * raw_yaw_velocity +
    (1.0 - velocity_filter_alpha_) * filtered_yaw_velocity_;
}

tf2::Transform SensorScanGenerationNode::computeOdomToChassis(
  const tf2::Transform & tf_odom_to_lidar, const rclcpp::Time & /*stamp*/)
{
  auto tf_lidar_to_chassis = getTransform(lidar_frame_, base_frame_, rclcpp::Time(0));

  bool is_identity =
    tf_lidar_to_chassis.getOrigin().length2() < 1e-10 &&
    std::abs(tf_lidar_to_chassis.getRotation().getW() - 1.0) < 1e-6;

  if (is_identity && !has_valid_lidar_to_chassis_) {
    return tf_odom_to_lidar;
  }

  if (!is_identity) {
    last_valid_lidar_to_chassis_ = tf_lidar_to_chassis;
    has_valid_lidar_to_chassis_ = true;
  } else {
    tf_lidar_to_chassis = last_valid_lidar_to_chassis_;
  }

  double roll, pitch, yaw;
  tf2::Matrix3x3(tf_lidar_to_chassis.getRotation()).getRPY(roll, pitch, yaw);
  tf2::Quaternion yaw_only;
  yaw_only.setRPY(0, 0, yaw);
  auto origin = tf_lidar_to_chassis.getOrigin();
  origin.setZ(0.0);
  tf_lidar_to_chassis.setOrigin(origin);
  tf_lidar_to_chassis.setRotation(yaw_only);

  auto tf_odom_to_chassis = tf_odom_to_lidar * tf_lidar_to_chassis;
  auto final_origin = tf_odom_to_chassis.getOrigin();
  final_origin.setZ(0.0);
  tf_odom_to_chassis.setOrigin(final_origin);

  double comp_angle = filtered_yaw_velocity_ * yaw_compensation_factor_;
  tf2::Quaternion comp_q;
  comp_q.setRPY(0, 0, comp_angle);
  tf_odom_to_chassis.setRotation(tf_odom_to_chassis.getRotation() * comp_q);

  return tf_odom_to_chassis;
}

tf2::Transform SensorScanGenerationNode::computeOdomToRobotBase(
  const tf2::Transform & tf_odom_to_lidar, const rclcpp::Time & stamp)
{
  auto lookup_time = stamp - rclcpp::Duration(0, 5000000);
  tf_lidar_to_robot_base_ = getTransform(lidar_frame_, robot_base_frame_, lookup_time);

  double roll, pitch, yaw;
  tf2::Matrix3x3(tf_lidar_to_robot_base_.getRotation()).getRPY(roll, pitch, yaw);
  tf2::Quaternion yaw_only;
  yaw_only.setRPY(0, 0, yaw);
  auto origin = tf_lidar_to_robot_base_.getOrigin();
  origin.setZ(0.0);
  tf_lidar_to_robot_base_.setOrigin(origin);
  tf_lidar_to_robot_base_.setRotation(yaw_only);

  return tf_odom_to_lidar * tf_lidar_to_robot_base_;
}

void SensorScanGenerationNode::odometryHandler(
  const nav_msgs::msg::Odometry::ConstSharedPtr & odometry_msg)
{
  tf2::Transform tf_odom_to_lidar;
  tf2::fromMsg(odometry_msg->pose.pose, tf_odom_to_lidar);

  const auto & stamp = odometry_msg->header.stamp;
  auto tf_odom_to_chassis = computeOdomToChassis(tf_odom_to_lidar, stamp);
  auto tf_odom_to_robot_base = computeOdomToRobotBase(tf_odom_to_lidar, stamp);

  publishTransform(
    tf_odom_to_chassis, odometry_msg->header.frame_id, base_frame_, stamp);
  publishOdometry(
    tf_odom_to_robot_base, odometry_msg->header.frame_id, robot_base_frame_, stamp);
}

void SensorScanGenerationNode::laserCloudAndOdometryHandler(
  const nav_msgs::msg::Odometry::ConstSharedPtr & odometry_msg,
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & pcd_msg)
{
  tf2::Transform tf_odom_to_lidar;
  tf2::fromMsg(odometry_msg->pose.pose, tf_odom_to_lidar);

  auto tf_odom_to_chassis = computeOdomToChassis(tf_odom_to_lidar, odometry_msg->header.stamp);

  sensor_msgs::msg::PointCloud2 out;
  pcl_ros::transformPointCloud(base_frame_, tf_odom_to_chassis.inverse(), *pcd_msg, out);
  pub_laser_cloud_->publish(out);
}

tf2::Transform SensorScanGenerationNode::getTransform(
  const std::string & target_frame, const std::string & source_frame, const rclcpp::Time & time)
{
  try {
    auto transform_stamped = tf_buffer_->lookupTransform(
      target_frame, source_frame, time, rclcpp::Duration::from_seconds(0.02));
    tf2::Transform transform;
    tf2::fromMsg(transform_stamped.transform, transform);
    return transform;
  } catch (tf2::ExtrapolationException &) {
    try {
      auto transform_stamped = tf_buffer_->lookupTransform(
        target_frame, source_frame, rclcpp::Time(0));
      tf2::Transform transform;
      tf2::fromMsg(transform_stamped.transform, transform);
      return transform;
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "TF fallback failed: %s. Returning identity.", ex.what());
      return tf2::Transform::getIdentity();
    }
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s. Returning identity.", ex.what());
    return tf2::Transform::getIdentity();
  }
}

void SensorScanGenerationNode::publishTransform(
  const tf2::Transform & transform, const std::string & parent_frame,
  const std::string & child_frame, const rclcpp::Time & stamp)
{
  geometry_msgs::msg::TransformStamped transform_msg;
  transform_msg.header.stamp = stamp;
  transform_msg.header.frame_id = parent_frame;
  transform_msg.child_frame_id = child_frame;
  transform_msg.transform = tf2::toMsg(transform);
  br_->sendTransform(transform_msg);
}

void SensorScanGenerationNode::publishOdometry(
  const tf2::Transform & transform, std::string parent_frame, const std::string & child_frame,
  const rclcpp::Time & stamp)
{
  nav_msgs::msg::Odometry out;
  out.header.stamp = stamp;
  out.header.frame_id = parent_frame;
  out.child_frame_id = child_frame;

  const auto & origin = transform.getOrigin();
  out.pose.pose.position.x = origin.x();
  out.pose.pose.position.y = origin.y();
  out.pose.pose.position.z = origin.z();
  out.pose.pose.orientation = tf2::toMsg(transform.getRotation());

  static tf2::Transform previous_transform;
  static auto previous_time = std::chrono::steady_clock::now();
  const auto current_time = std::chrono::steady_clock::now();

  const double dt =
    std::chrono::duration_cast<std::chrono::nanoseconds>(current_time - previous_time).count() *
    1e-9;

  if (dt > 0) {
    const auto linear_velocity = (transform.getOrigin() - previous_transform.getOrigin()) / dt;

    const tf2::Quaternion q_diff =
      transform.getRotation() * previous_transform.getRotation().inverse();
    const auto angular_velocity = q_diff.getAxis() * q_diff.getAngle() / dt;

    out.twist.twist.linear.x = linear_velocity.x();
    out.twist.twist.linear.y = linear_velocity.y();
    out.twist.twist.linear.z = linear_velocity.z();
    out.twist.twist.angular.x = angular_velocity.x();
    out.twist.twist.angular.y = angular_velocity.y();
    out.twist.twist.angular.z = angular_velocity.z();
  }

  previous_transform = transform;
  previous_time = current_time;

  pub_chassis_odometry_->publish(out);
}

}  // namespace sensor_scan_generation

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(sensor_scan_generation::SensorScanGenerationNode)
