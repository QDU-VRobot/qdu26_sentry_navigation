// Copyright 2026 QDU RoboMaster Team
//
// Licensed under the Apache License, Version 2.0 (the "License");

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

class OdometryToTFNode : public rclcpp::Node
{
public:
  OdometryToTFNode()
  : Node("odometry_to_tf")
  {
    this->declare_parameter("odom_topic", "/odin1/odometry");
    odom_topic_ = this->get_parameter("odom_topic").as_string();

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 10,
      std::bind(&OdometryToTFNode::odomCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Odometry to TF node started, subscribing to: %s",
                odom_topic_.c_str());
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    geometry_msgs::msg::TransformStamped transform;
    transform.header = msg->header;
    transform.child_frame_id = msg->child_frame_id;
    transform.transform.translation.x = msg->pose.pose.position.x;
    transform.transform.translation.y = msg->pose.pose.position.y;
    transform.transform.translation.z = msg->pose.pose.position.z;
    transform.transform.rotation = msg->pose.pose.orientation;

    tf_broadcaster_->sendTransform(transform);
  }

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  std::string odom_topic_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdometryToTFNode>());
  rclcpp::shutdown();
  return 0;
}
