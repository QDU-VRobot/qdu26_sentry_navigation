# Odin1 Integration Package

## 概述

此包用于集成 Odin1 雷达到 ROS2 导航系统,提供以下功能:

1. **TF 反转节点** (`tf_inverter_node`): 将 Odin1 发布的 `odom->map` TF 反转为 Nav2 标准的 `map->odom` TF
2. **点云融合节点** (`pointcloud_fusion_node`): 融合 Odin1 和 Mid-360 两个雷达的点云数据

## 节点说明

### 1. TF Inverter Node

**功能**: 监听 Odin1 发布的 `odom->map` TF,反转后发布标准的 `map->odom` TF 供 Nav2 使用

**参数**:
- `source_frame` (string, default: "odom"): 源坐标系
- `target_frame` (string, default: "map"): 目标坐标系
- `inverted_source_frame` (string, default: "map"): 反转后的源坐标系
- `inverted_target_frame` (string, default: "odom"): 反转后的目标坐标系
- `publish_rate` (double, default: 20.0): 发布频率 (Hz)

### 2. PointCloud Fusion Node

**功能**: 将 Odin1 和 Mid-360 的点云转换到统一坐标系并融合

**订阅话题**:
- `odin1/cloud_raw` (sensor_msgs/PointCloud2): Odin1 原始点云
- `/livox/lidar` (sensor_msgs/PointCloud2): Mid-360 点云

**发布话题**:
- `fused_pointcloud` (sensor_msgs/PointCloud2): 融合后的点云

**参数**:
- `odin1_topic` (string, default: "odin1/cloud_raw"): Odin1 点云话题
- `mid360_topic` (string, default: "/livox/lidar"): Mid-360 点云话题
- `output_topic` (string, default: "fused_pointcloud"): 输出点云话题
- `output_frame` (string, default: "base_footprint"): 输出坐标系
- `publish_rate` (double, default: 10.0): 发布频率 (Hz)
- `sync_tolerance` (double, default: 0.1): 时间同步容差 (秒)

## 静态 TF 配置

在 `robot_description_launch.py` 中已集成静态 TF 发布器,用于定义 `front_mid360` 到 `odin1_frame` 的坐标变换。

**修改方法**:
编辑 `/home/alis/codes/qdu26_sentry_mix/src/qdu2026_robot_description/launch/robot_description_launch.py` 中的参数:

```python
arguments=[
    "--x", "0.0",      # X 平移 (米)
    "--y", "0.0",      # Y 平移 (米)
    "--z", "0.0",      # Z 平移 (米)
    "--roll", "0.0",   # Roll 旋转 (弧度)
    "--pitch", "0.0",  # Pitch 旋转 (弧度)
    "--yaw", "0.0",    # Yaw 旋转 (弧度)
    "--frame-id", "odin1_frame",
    "--child-frame-id", "front_mid360"
]
```

## 编译

```bash
cd /home/alis/codes/qdu26_sentry_mix
colcon build --packages-select odin1_integration
source install/setup.bash
```

## 使用方法

### 导航模式

使用原有的启动脚本即可,已自动集成:

```bash
./nav_start.sh <world>
```

例如:
```bash
./nav_start.sh yilou
```

### 单独测试节点

如需单独测试 Odin1 集成节点:

```bash
ros2 launch odin1_integration odin1_integration_launch.py
```

## 架构说明

### 旧架构 (已移除)
- point_lio: 提供里程计和 `cloud_registered` 点云
- small_gicp_relocalization: 订阅 `cloud_registered`,发布 `map->odom` TF

### 新架构
- Odin1 雷达: 发布 `odin1/odometry` 和 `odom->map` TF
- tf_inverter_node: 反转为 `map->odom` TF
- pointcloud_fusion_node: 融合 Odin1 和 Mid-360 点云
- 融合点云可用于 Nav2 的障碍物检测

## TF 树结构

```
map (Odin1 地图坐标系)
 └─ odom (Odin1 里程计坐标系,由 tf_inverter 反转发布)
     └─ base_footprint (机器人底盘)
         ├─ odin1_frame (Odin1 雷达坐标系)
         └─ front_mid360 (Mid-360 雷达坐标系,通过静态 TF 连接到 odin1_frame)
```

## 注意事项

1. **Odin1 驱动**: 确保 Odin1 驱动正常运行并发布以下内容:
   - 话题: `odin1/cloud_raw`, `odin1/odometry`
   - TF: `odom->map`

2. **坐标系校准**: 根据实际机器人配置,调整 `front_mid360` 到 `odin1_frame` 的静态 TF 参数

3. **时间同步**: 两个雷达的时间戳应尽量同步,否则融合效果会受影响

4. **性能**: 点云融合节点默认 10Hz,可根据需要调整 `publish_rate` 参数

## 故障排查

### TF 反转节点无输出
- 检查 Odin1 是否正常发布 `odom->map` TF: `ros2 run tf2_ros tf2_echo odom map`
- 查看节点日志: `ros2 node info /tf_inverter`

### 点云融合无输出
- 检查两个雷达话题是否正常: `ros2 topic hz odin1/cloud_raw` 和 `ros2 topic hz /livox/lidar`
- 检查 TF 树是否完整: `ros2 run tf2_tools view_frames`
- 查看节点日志: `ros2 node info /pointcloud_fusion`

### 导航异常
- 确认 `map->odom` TF 正常发布: `ros2 run tf2_ros tf2_echo map odom`
- 检查融合点云是否发布: `ros2 topic hz fused_pointcloud`
- 在 RViz 中可视化 TF 树和点云
