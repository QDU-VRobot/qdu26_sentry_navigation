// Copyright 2026 QDU RoboMaster Team
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

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

class TFInverterNode : public rclcpp::Node
{
public:
  TFInverterNode()
  : Node("tf_inverter_node")
  {
    this->declare_parameter("source_frame", "odom");
    this->declare_parameter("target_frame", "map");
    this->declare_parameter("inverted_source_frame", "map");
    this->declare_parameter("inverted_target_frame", "odom");
    this->declare_parameter("publish_rate", 20.0);

    source_frame_ = this->get_parameter("source_frame").as_string();
    target_frame_ = this->get_parameter("target_frame").as_string();
    inverted_source_frame_ = this->get_parameter("inverted_source_frame").as_string();
    inverted_target_frame_ = this->get_parameter("inverted_target_frame").as_string();
    double publish_rate = this->get_parameter("publish_rate").as_double();

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate),
      std::bind(&TFInverterNode::timerCallback, this));

    RCLCPP_INFO(this->get_logger(),
      "TF Inverter started: listening %s->%s, publishing %s->%s at %.1f Hz",
      source_frame_.c_str(), target_frame_.c_str(),
      inverted_source_frame_.c_str(), inverted_target_frame_.c_str(),
      publish_rate);
  }

private:
  void timerCallback()
  {
    try {
      // Listen to odom->map from Odin1
      auto transform_stamped = tf_buffer_->lookupTransform(
        target_frame_, source_frame_, tf2::TimePointZero);

      // Manually invert the transform
      geometry_msgs::msg::TransformStamped inverted_transform;
      inverted_transform.header.stamp = this->now();
      inverted_transform.header.frame_id = inverted_source_frame_;
      inverted_transform.child_frame_id = inverted_target_frame_;

      // Invert translation: t_inv = -R^T * t
      tf2::Quaternion q(
        transform_stamped.transform.rotation.x,
        transform_stamped.transform.rotation.y,
        transform_stamped.transform.rotation.z,
        transform_stamped.transform.rotation.w);

      tf2::Matrix3x3 m(q);
      tf2::Vector3 t(
        transform_stamped.transform.translation.x,
        transform_stamped.transform.translation.y,
        transform_stamped.transform.translation.z);

      tf2::Vector3 t_inv = m.transpose() * (-t);

      inverted_transform.transform.translation.x = t_inv.x();
      inverted_transform.transform.translation.y = t_inv.y();
      inverted_transform.transform.translation.z = t_inv.z();

      // Invert rotation: q_inv = q*
      inverted_transform.transform.rotation.x = -q.x();
      inverted_transform.transform.rotation.y = -q.y();
      inverted_transform.transform.rotation.z = -q.z();
      inverted_transform.transform.rotation.w = q.w();

      // Publish map->odom for Nav2
      tf_broadcaster_->sendTransform(inverted_transform);

    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "Could not transform %s to %s: %s",
        source_frame_.c_str(), target_frame_.c_str(), ex.what());
    }
  }

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string source_frame_;
  std::string target_frame_;
  std::string inverted_source_frame_;
  std::string inverted_target_frame_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TFInverterNode>());
  rclcpp::shutdown();
  return 0;
}
