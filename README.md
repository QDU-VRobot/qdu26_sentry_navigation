# 哨兵导航传承文档（qdu26_sentry_mix）

---

## 0. 概述

ROS2 Humble 哨兵机器人导航工程，基于 [SMBU PolarBear 队伍开源工程](https://github.com/SMBU-PolarBear-Robotics-Team/qdu2026_sentry_nav) 改造而来。

- 雷达：**Odin1**（自带 SLAM/重定位） + **Livox MID360**（云台前视）
- SLAM：使用 Odin1 原生重定位，输出 `map → odom`
- 里程计：Odin1 发布 `odom → odin1_base_link`
- 导航：Nav2 + 自研代价地图层 + PB Pursuit 控制器
- 通信：与下位机 ros2_libxr 通过 ROS 2 topic 对接

---

## 1. 仓库结构

```
qdu26_sentry_mix/
├── nav_start.sh              # 等待 odin1 重定位完成后再启动导航
├── nav_start_wr.sh           # 不等重定位，直接启动（即无重定位导航）
├── save_pcd.sh               # 累积 fused 点云保存为 PCD（odin1的SLAM启动时的一个建pcd的脚本）
├── HANDOVER.md               # ← 你正在看的这份
└── src/
    ├── qdu2026_sentry_nav/                # 主导航工程
    │   ├── qdu2026_nav_bringup/           # 启动入口、地图、参数、行为树
    │   │   ├── launch/                    # rm_navigation_launch / bringup / localization / navigation / slam / rviz
    │   │   ├── config/nav2_params.yaml    # 全部节点的统一参数文件
    │   │   ├── behavior_trees/*.xml       # Nav2 行为树（恢复行为在这里调）
    │   │   ├── map/<world>.{pgm,yaml}     # 静态栅格地图
    │   │   └── pcd/<world>.pcd            # 先验点云（在没有odin1时，用于重定位，现在有了odin1重定位，pcd点云已经不再是必须）
    │   ├── odin1_integration/             # odin1融合的核心节点
    │   │   ├── src/pointcloud_fusion_node.cpp     # 双雷达点云融合（最重要）
    │   │   ├── src/approach_velocity_controller.cpp # 接近目标减速（防止机器人过快，目标点检测容忍度低，机器人到目标点后发生绕圈振荡）
    │   │   ├── src/odometry_to_tf_node.cpp        # odom→TF 转换
    │   │   ├── src/tf_inverter_node.cpp           # TF 取反（odin1的TF链路是odom->map，和正常的相反）
    │   │   ├── src/goal_status_node.cpp           # 目标点状态汇报（和决策树通信用的一个测试代码）
    │   │   └── launch/odin1_integration_launch.py
    │   ├── sensor_scan_generation/        # 点云去畸变 + odom→base TF + 云台旋转补偿
    │   ├── terrain_analysis/              # 局部地形分析（local costmap 输入）
    │   ├── terrain_analysis_ext/          # 远距离地形分析（global costmap 输入）
    │   ├── pointcloud_to_laserscan/       # 把 terrain_map 变 LaserScan 给 costmap
    │   ├── pb_nav2_plugins/               # IntensityVoxelLayer、BackUpFreeSpace 等插件
    │   ├── pb_omni_pid_pursuit_controller/# 全向 PID 跟踪控制器（FollowPath 插件）
    │   ├── fake_vel_transform/            # gimbal_yaw_fake 帧 + 旋转坐标系下的速度变换
    │   ├── livox_ros_driver2/             # MID360 驱动
    │   └── loam_interface/                # 占位，无实际用途
    ├── qdu2026_robot_description/         # 机器人 URDF/xmacro，发布 robot_description + 静态 TF
    ├── pcd2pgm/                           # PCD 点云转栅格 pgm 地图
    └── sdformat_tools/                    # xmacro→sdf→urdf 工具链
```

> `qdu2026_robot_description`、`pcd2pgm`、`sdformat_tools` 是子模块。

---

## 2. 数据链路（必看）

```
                ┌────────────────────────┐
   Odin1 驱动 ──┤ odin1/cloud_raw        │
                │ odin1/odometry         │
                │ TF: map→odom           │
                │ TF: odom→odin1_base_link│
                └─────────────┬──────────┘
                              │
              ┌───────────────┴────────────────────────┐
              │                                        │
              ▼                                        ▼
       MID360 驱动 ──── /livox/lidar               (各路点云)
                              │
                              ▼
            ┌────────────────────────────────┐
            │ pointcloud_fusion_node         │  ← odin1_integration
            │ ・两路点云都 TF 到 base_footprint│
            │ ・做自过滤（去掉自己车身的点） │
            │ ・输出 fused_pointcloud (RELIABLE) │
            └─────────────┬──────────────────┘
                          │
        ┌─────────────────┼───────────────────────────┐
        ▼                 ▼                           ▼
 sensor_scan_generation  terrain_analysis    terrain_analysis_ext
 (TF 发布 + 去畸变)      (5m 半径)           (20m 半径)
        │                 │                           │
        │            terrain_map               terrain_map_ext
        │                 │                           │
        │                 ▼                           ▼
        │          local_costmap (intensity_voxel)  global_costmap
        │                 │                           │
        ▼                 ▼                           ▼
   /odometry       Nav2 controller_server / planner_server
                          │
                          ▼
                  cmd_vel_nav2_result
                          │
                  fake_vel_transform   ← 把 gimbal_yaw_fake 系下的速度变到 chassis 系
                          │
                          ▼
                       cmd_vel  ──→ ros2_libxr ──→ 底盘
```

要点：
- 所有 costmap 输入点云的 frame 都已经在 `pointcloud_fusion_node` 里转到 `base_footprint`，这样云台转动不会污染 costmap。
- `fake_vel_transform` 发布 `base_footprint → gimbal_yaw_fake` 的纯 yaw TF，让 Nav2 在「不随云台转的」坐标系里规划。

---

## 3. TF 树

```
map
 └── odom                              ← Odin1 重定位发布
      ├── odin1_base_link              ← Odin1 里程计
      │     └── front_mid360 (静态)    ← robot_description_launch.py
      └── base_footprint               ← sensor_scan_generation 发布（odom→base_footprint）
            ├── gimbal_yaw_fake (动态) ← fake_vel_transform（只有底盘 yaw）
            └── chassis
                  ├── gimbal_yaw_odom → gimbal_pitch_odom → gimbal_yaw → front_mid360
                  └── wheels / armor / ...
```

关键约束：
- `lidar_frame = odin1_base_link`，必须与 Odin1 里程计 child_frame_id 一致。
- Nav2 里 `robot_base_frame = gimbal_yaw_fake`（绕过云台）。
- costmap 里 `sensor_frame = base_footprint`，**绝对不能写云台帧**。

---

## 4. 编译 & 环境

### 4.1 系统依赖

```bash
sudo apt update
sudo apt install -y git python3-pip git-lfs
pip3 install xmacro
```

本代码运行环境为 ROS 2 Humble + Ubuntu 22.04，本教程不确保其他ROS2版本与乌班图版本下能正常跑通。

### 4.2 三个外部工程（不在本仓库里）

启动脚本会 source 它们，缺一不可：

| 路径 | 作用 |
|---|---|
| `/opt/ros/humble` | ROS 2 Humble |
| `~/codes/ros2_libxr` | 与下位机通信（cmd_vel / 串口） |
| `~/codes/catkin_ws` | Odin1 驱动 `odin_ros_driver` |
| `~/codes/qdu26_sentry_mix` | 本仓库 |

下面三个仓库需要你自己 clone 编译：
- `ros2_libxr`：上下位机通信
- `odin_ros_driver`：Odin1 雷达驱动（重定位/里程计输出）
- 本仓库

### 4.3 编译

```bash
cd ~/codes/qdu26_sentry_mix
rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

`--symlink-install` 之后改 yaml/launch 不用重新 colcon build，重启节点就生效。

---

## 5. 启动流程

### 5.1 一键启动（推荐）

```bash
cd ~/codes/qdu26_sentry_mix
./nav_start.sh rmuc          # rmuc / rmul / kongbai 等，对应 map/<name>.yaml
```

`nav_start.sh` 做的事：
1. 启动 ros2_libxr（下位机通信）
2. 清空 cmd_vel
3. 启动 Odin1，**等到日志里出现 `relocalization success`** 才继续
4. 启动 robot_description（URDF + 静态 TF）
5. 起一个 `map→odom` 静态 TF（先占位，Odin1 重定位成功后会接管）
6. 启动 Nav2

退出时 trap 会杀掉所有子进程。

### 5.2 无重定位启动（不等重定位）

```bash
./nav_start_wr.sh rmuc
```

区别：不等待 `relocalization success`，启动更快，但 Odin1 没定位上时车就在原点附。


### 5.3 单独建图（先验点云重定位用）

```bash
#去catkin_ws中（odin1驱动文件）,将参数文件中，odin1工作模式启动为SLAM模式
#启动odin1驱动（另外odin1用于重定位的地图是它自己的私有格式，而不是pcd点云格式，所以odin1自己的地图需要单独保存，具体步骤在odin1的文档中有，此处不再赘述）
./save_pcd.sh <新世界名>.pcd 0.05
# 开车跑一圈
```
`0.05`是指的点云分辨率，值越小，点云数量越多，pcd文件也越大
`save_pcd.sh` 订阅 `/odin1/cloud_slam`，累积+体素降采样，Ctrl+C 保存。

> 因为重定位用odin1了，所以建出来的pcd，目前仅用于转为pgm二维栅格地图，所以没必要太精细

### 5.4 PCD → PGM（生成静态地图）

```bash
#首先将pcd点云移至src/pcd2pgm/pcd
source install/setup.bash
#然后修改src/pcd2pgm/config/pcd2pgm.yaml参数文件的pcd文件路径
ros2 launch pcd2pgm pcd2pgm_launch.py
#然后看看rviz2里面的栅格地图，如果角度，高度不对，就改pcd2pgm.yaml中的参数
ros2 run nav2_map_server map_saver_cli -f <新世界名>
# 生成的 <新世界名>.{pgm,yaml} 移到 qdu2026_nav_bringup/map/
```

---

## 6. 关键参数（nav2_params.yaml）

文件路径：`src/qdu2026_sentry_nav/qdu2026_nav_bringup/config/nav2_params.yaml`

### 6.1 pointcloud_fusion（双雷达融合）

| 参数 | 当前值 | 说明 |
|---|---|---|
| `output_frame` | `base_footprint` | 必须是 `base_footprint`，不能用云台帧 |
| `enable_deskew` | `true` | 用 IMU 做点云去畸变 |
| `max_sync_diff_sec` | `0.15` | mid360 与 odin1 时间戳同步窗口，太小会丢帧 |
| `mid360_queue_size` | `5` | 缓存几帧 mid360 等 odin1 |
| `publish_reliable` | `true` | 必须 RELIABLE，下游 terrain_analysis 是 RELIABLE 订阅 |
| `self_filter_half_x/y` | `0.35` | 自过滤盒子半长（去掉车身点云） |
| `self_filter_height` | `0.6` | 自过滤盒子高度 |
| `self_filter_min_z` | `-0.4` | 自过滤盒子下边界 |

### 6.2 sensor_scan_generation（云台旋转补偿）

> 哨兵在云台扫描时，整个的地图会旋转一个角度，云台转的越快，这个角度偏离的越大，在此处进行一个补偿（这个问题我解决了好久都没解决，只能用这种补偿的方法，或者和电控商量一下，哨兵在导航移动时，让云台不动，到了目标点才扫描）

| 参数 | 当前值 | 说明 |
|---|---|---|
| `yaw_compensation_factor` | `0.35` | 云台 yaw 角速度的补偿系数，过大会反向飞 |
| `velocity_filter_alpha` | `0.2` | IMU 角速度低通滤波，越小越平滑 |

> **重要**：补偿用的是 `/livox/imu` 的绝对角速度，**不要**改成 joint_states 相对角度，否则底盘转动会误补偿。

### 6.3 terrain_analysis / terrain_analysis_ext

| 参数 | 当前值 | 说明 |
|---|---|---|
| `decayTime` | `0.15` | 点云时间窗口，太大有残影，太小看不到障碍 |
| `clearingDis` | `5.0` / `20.0` | local 5 米、global 20 米清除半径 |
| `vehicleHeight` | `0.8` | 高于此值的点云不参与障碍判断 |
| `useSorting` | `true` | 开启后能上坡（地面点用分位数估计） |

### 6.4 controller_server（FollowPath 插件）

| 参数 | 当前值 | 说明 |
|---|---|---|
| `controller_frequency` | `30.0` | 控制频率，CPU 紧张时降到 20 |
| `v_linear_max/min` | `±0.7` | 线速度上限，**与 velocity_smoother 同步** |
| `v_angular_max/min` | `±0.2` | 角速度上限（云台已自旋，底盘不需要大） |
| `min_approach_linear_velocity` | `0.1` | 接近目标的最低速度兜底，**外部限速要低于这个值** |
| `curvature_max` / `reduction_ratio_at_high_curvature` | `0.5` / `0.7` | 高曲率减速比例 |

### 6.5 velocity_smoother

| 参数 | 当前值 | 说明 |
|---|---|---|
| `max_velocity` | `[0.7, 0.7, 0.6]` | xyz 上限，必须 ≥ controller `v_linear_max` |
| `max_accel` | `[0.8, 0.8, 0.8]` | 加速度限制 |
| `max_decel` | `[-1.5, -1.5, -1.0]` | 减速度限制 |

### 6.6 approach_velocity_controller（自定义）

| 参数 | 当前值 | 说明 |
|---|---|---|
| `slow_down_distance` | `0.15` | 距目标多远开始减速 |
| `min_speed` | `0.3` | 接近时最低速度 |
| `normal_speed` | `0.7` | 正常巡航速度 |

### 6.7 costmap

- `inflation_radius`：局部 0.6，全局 0.6
- `cost_scaling_factor`：局部 8.0，全局 15.0
- `robot_radius`：0.15
- `intensity_voxel_layer`：基于 intensity 过滤地形点云，`min_obstacle_intensity: 0.1`（intensity=0 的点不当障碍）

### 6.8 fake_vel_transform

- `init_spin_speed: 0.0`：上电默认自转角速度（rad/s）
- `cmd_spin_topic: cmd_spin`：上层可以单独发布自旋指令

### 6.9 行为树（恢复动作）

`behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml`：

```xml
<BackUp backup_dist="0.3" backup_speed="0.8"/>
```

后退恢复距离 0.3m，速度 0.8 m/s。卡住时会自动后退一下再重新规划。



---

## 8. 联系方式 / 参考

- 本人QQ：1376982098
- 上游开源：https://github.com/SMBU-PolarBear-Robotics-Team/qdu2026_sentry_nav
- Odin1 官方资料：https://manifoldtechltd.github.io/wiki/odin_series/odin1/
- Odin1 驱动代码: https://github.com/manifoldsdk/odin_ros_driver
