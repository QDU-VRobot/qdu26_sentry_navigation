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
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/common/transforms.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_eigen/tf2_eigen.hpp"

class PointCloudFusionNode : public rclcpp::Node
{
public:
  PointCloudFusionNode()
  : Node("pointcloud_fusion_node")
  {
    this->declare_parameter("odin1_topic", "odin1/cloud_raw");
    this->declare_parameter("mid360_topic", "/livox/lidar");
    this->declare_parameter("output_topic", "fused_pointcloud");
    this->declare_parameter("output_frame", "odin1_base_link");

    auto odin1_topic = this->get_parameter("odin1_topic").as_string();
    auto mid360_topic = this->get_parameter("mid360_topic").as_string();
    auto output_topic = this->get_parameter("output_topic").as_string();
    output_frame_ = this->get_parameter("output_frame").as_string();

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    rmw_qos_profile_t qos = rmw_qos_profile_sensor_data;
    auto sensor_qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(qos), qos);

    odin1_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      odin1_topic, sensor_qos,
      std::bind(&PointCloudFusionNode::odin1Callback, this, std::placeholders::_1));

    mid360_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      mid360_topic, sensor_qos,
      std::bind(&PointCloudFusionNode::mid360Callback, this, std::placeholders::_1));

    fused_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic, 10);

    RCLCPP_INFO(this->get_logger(), "PointCloud Fusion: %s + %s -> %s",
      odin1_topic.c_str(), mid360_topic.c_str(), output_topic.c_str());
  }

private:
  // Pre-transform mid360 cloud in its own callback (heavy work done here)
  void mid360Callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr xyz(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *xyz);

    if (msg->header.frame_id != output_frame_) {
      try {
        auto tf = tf_buffer_->lookupTransform(output_frame_, msg->header.frame_id, tf2::TimePointZero);
        Eigen::Affine3d eigen_tf = tf2::transformToEigen(tf.transform);
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud(*xyz, *transformed, eigen_tf);
        xyz = transformed;
      } catch (tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
          "mid360 TF: %s", ex.what());
        return;
      }
    }

    // Convert to XYZI and cache
    auto xyzi = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    xyzi->reserve(xyz->size());
    for (const auto & p : *xyz) {
      pcl::PointXYZI pi;
      pi.x = p.x; pi.y = p.y; pi.z = p.z; pi.intensity = 0.0f;
      xyzi->push_back(pi);
    }

    std::lock_guard<std::mutex> lock(mutex_);
    mid360_cache_ = xyzi;
  }

  // Odin1 callback: lightweight — just convert odin1 + concat cached mid360
  void odin1Callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr odin_xyz(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *odin_xyz);

    std::lock_guard<std::mutex> lock(mutex_);

    size_t mid_size = mid360_cache_ ? mid360_cache_->size() : 0;
    pcl::PointCloud<pcl::PointXYZI>::Ptr fused(new pcl::PointCloud<pcl::PointXYZI>);
    fused->reserve(odin_xyz->size() + mid_size);

    for (const auto & p : *odin_xyz) {
      pcl::PointXYZI pi;
      pi.x = p.x; pi.y = p.y; pi.z = p.z; pi.intensity = 0.0f;
      fused->push_back(pi);
    }

    if (mid360_cache_) {
      *fused += *mid360_cache_;
    }

    sensor_msgs::msg::PointCloud2 out;
    pcl::toROSMsg(*fused, out);
    out.header.stamp = msg->header.stamp;
    out.header.frame_id = output_frame_;
    fused_pub_->publish(out);
  }

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr odin1_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr mid360_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr fused_pub_;

  std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> mid360_cache_;
  std::string output_frame_;
  std::mutex mutex_;
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
