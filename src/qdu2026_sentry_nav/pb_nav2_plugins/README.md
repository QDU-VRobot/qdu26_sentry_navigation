# pb_nav2_plugins

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![Build and Test](https://github.com/SMBU-PolarBear-Robotics-Team/pb_nav2_plugins/actions/workflows/ci.yml/badge.svg)](https://github.com/SMBU-PolarBear-Robotics-Team/pb_nav2_plugins/actions/workflows/ci.yml)
[![pre-commit](https://img.shields.io/badge/pre--commit-enabled-brightgreen?logo=pre-commit)](https://github.com/pre-commit/pre-commit)

![PolarBear Logo](https://raw.githubusercontent.com/SMBU-PolarBear-Robotics-Team/.github/main/.docs/image/polarbear_logo_text.png)

## 1. Overview

`pb_nav2_plugins` 是一个用于扩展 `Navigation2`（Nav2）框架的插件库，当前提供提供了一些额外的行为逻辑和代价地图层以增强机器人在导航过程中灵活性。

## 2. Plugins

### 2.1 Behaviors

#### 2.1.1 BackUpFreeSpace

`BackUpFreeSpace` 插件是一个用于在机器人的导航过程中执行后退行为的插件，主要功能：

1. **动态调整半径**：通过调整搜索的半径范围，机器人在限定的范围内找到足够的自由空间后退。
2. **退回到自由空间**：机器人根据当前的代价图，找出最接近的自由空间并退回到该位置。

**Parameters:**

- `robot_radius`: 机器人半径，用于确定搜索自由空间时的范围（default：0.1 m）。
- `max_radius`: 搜索自由空间时的最大半径范围（default：1.0 m）。
- `service_name`: 获取代价图的服务名称（default："local_costmap/get_costmap"）。
- `free_threshold`: 定义自由空间的阈值，即自由空间中足够数量的点才能视为有效（default：5）。
- `visualize`: 是否启用可视化功能。启用后会在 RViz 中显示自由空间和目标位置（default：false）。

**Example:**

```yaml
behavior_server:
ros__parameters:
   use_sim_time: true
   local_costmap_topic: local_costmap/costmap_raw
   global_costmap_topic: global_costmap/costmap_raw
   local_footprint_topic: local_costmap/published_footprint
   global_footprint_topic: global_costmap/published_footprint
   cycle_frequency: 10.0
   behavior_plugins: ["spin", "backup", "drive_on_heading", "assisted_teleop", "wait"]
   spin:
      plugin: "nav2_behaviors/Spin"
   backup:
      plugin: "pb_nav2_behaviors/BackUpFreeSpace"
   drive_on_heading:
      plugin: "nav2_behaviors/DriveOnHeading"
   wait:
      plugin: "nav2_behaviors/Wait"
   assisted_teleop:
      plugin: "nav2_behaviors/AssistedTeleop"
   local_frame: odom
   global_frame: map
   robot_base_frame: gimbal_yaw_fake
   transform_tolerance: 0.1
   simulate_ahead_time: 2.0
   max_rotational_vel: 1.0
   min_rotational_vel: 0.4
   rotational_acc_lim: 3.2
   # params for pb_nav2_behaviors/BackUpFreeSpace
   robot_radius: 0.2
   max_radius: 3.5
   service_name: "global_costmap/get_costmap"
   free_threshold: 5
   visualize: True
```

### 2.2 Layers

#### 2.2.1 IntensityVoxelLayer

`IntensityVoxelLayer` 是一个用于处理点云数据中障碍物强度信息的代价地图层。它可以根据点云数据中的强度值来标记障碍物，并将这些障碍物信息添加到代价地图中，本插件推荐配合 [terrain_analysis](https://github.com/SMBU-PolarBear-Robotics-Team/terrain_analysis) 功能包使用。

**Parameters:**

Common to [costmap-plugins/voxel.html](https://docs.nav2.org/configuration/packages/costmap-plugins/voxel.html)。

主要区别在于添加了 `min_obstacle_intensity` 和 `max_obstacle_intensity` 参数，注意此参数使用域在 `obstacle_layer` 下，而非 `observation_sources` 下。

**Example:**

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      use_sim_time: true
      update_frequency: 10.0
      publish_frequency: 5.0
      global_frame: odom
      robot_base_frame: gimbal_yaw_fake
      rolling_window: true
      width: 5
      height: 5
      resolution: 0.05
      robot_radius: 0.2
      plugins: ["static_layer", "intensity_voxel_layer", "inflation_layer"]
      intensity_voxel_layer:
        plugin: pb_nav2_costmap_2d::IntensityVoxelLayer
        enabled: true
        track_unknown_space: true
        footprint_clearing_enabled: true
        publish_voxel_map: false
        combination_method: 1
        mark_threshold: 0
        origin_z: 0.0
        unknown_threshold: 5
        z_resolution: 0.05
        z_voxels: 16
        min_obstacle_height: 0.0
        max_obstacle_height: 2.0
        min_obstacle_intensity: 0.1
        max_obstacle_intensity: 2.0
        observation_sources: terrain_map
        terrain_map:
          data_type: PointCloud2
          topic: /terrain_map
          min_obstacle_height: 0.0
          max_obstacle_height: 2.0
          obstacle_max_range: 5.0
          obstacle_min_range: 0.2
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 4.0
        inflation_radius: 0.7
      always_send_full_costmap: False
```

## Acknowledgements

The Initial Developer of some parts of the repository (`BackUpFreeSpace`, `IntensityVoxelLayer`, `IntensityVoxelLayer`), which are copied from, derived from, or
inspired by @PolarisXQ [SCURM_SentryNavigation](https://github.com/PolarisXQ/SCURM_SentryNavigation/tree/master/nav2_plugins/behavior_ext_plugins), @ros-navigation [navigation2](https://github.com/ros-navigation).
All Rights Reserved.
