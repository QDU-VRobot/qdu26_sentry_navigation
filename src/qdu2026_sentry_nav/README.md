# qdu26_sentry

本包基于 Shenzhen SMBU PolarBear Robotics Team 的导航开源项目 (https://github.com/SMBU-PolarBear-Robotics-Team/qdu2026_sentry_nav)，已移除仿真与无关模块，仅保留并适配实车导航相关功能。感谢原项目的贡献。

## 目录结构（精简说明）

- fake_vel_transform：虚拟速度参考坐标系（用于云台扫描模式自旋）
- ign_sim_pointcloud_tool：仿真点云处理工具（保留仓库结构中但实车无需使用）
- livox_ros_driver2：Livox 激光雷达驱动
- loam_interface：里程计算法接口（point_lio 等）
- pb_teleop_twist_joy：手柄控制节点
- qdu2026_nav_bringup：启动文件（实车使用）
- qdu2026_sentry_nav：包描述
- pb_omni_pid_pursuit_controller：路径跟踪控制器
- point_lio：里程计实现
- pointcloud_to_laserscan：点云到 LaserScan 转换（仅 SLAM/特殊场景）
- sensor_scan_generation：点云相关坐标变换
- small_gicp_relocalization：重定位模块
- terrain_analysis / terrain_analysis_ext：地形分析模块


## 实车使用说明（已移除仿真部分）

本仓库面向实车部署，下面示例以在真实车辆上启动导航为主。启动前请确认：
- 所有硬件驱动已启动（IMU、里程计、LiDAR、云台等）。
- 对应的参数文件、地图（world）、先验点云已准备并路径正确。
- 若使用 robot_state_publisher 发布机器人关节 TF，请根据实际硬件选择是否启用。

示例启动命令（实车模式）：

```bash
ros2 launch qdu2026_nav_bringup rm_navigation_launch.py \
world:=<YOUR_WORLD_NAME> \
map:=<ABS_PATH_TO_MAP_YAML> \
prior_pcd_file:=<ABS_PATH_TO_PRIOR_PCD> \
params_file:=<ABS_PATH_TO_PARAMS_YAML> \
rviz_config_file:=<ABS_PATH_TO_RVIZ_CONFIG> \
use_robot_state_pub:=<True|False> \
autostart:=True
```

常用示例（若使用默认实车参数文件并在工作区中）：

```bash
ros2 launch qdu2026_nav_bringup rm_navigation_launch.py \
world:=<YOUR_WORLD_NAME> \
use_robot_state_pub:=False
```

如果只需启动最小导航栈

```bash
ros2 launch qdu2026_nav_bringup rm_navigation_launch.py \
world:=<YOUR_WORLD_NAME> \
use_robot_state_pub:=True
```

## 启动参数（实车相关）

以下列出常用且与实车部署直接相关的参数：

- use_sim_time：是否使用仿真时钟（实车请设为 False）
- slam：是否启用建图模式（实车通常为 False）
- world：地图/场景名称（对应 map / prior_pcd_file 的自动填充）
- map：要加载的地图 YAML 文件路径（推荐使用绝对路径）
- prior_pcd_file：先验点云文件路径（若使用）
- params_file：Nav2 与其它节点的参数文件路径
- rviz_config_file：RViz 配置文件路径
- autostart：是否自动启动 Nav2
- use_composition：是否使用可组合节点方式启动
- use_robot_state_pub：是否由本包发布 robot_state（实车常由独立模块提供，推荐设为 False）

## 地图与先验点云

- 若需要保存或生成地图，请在受控环境中运行建图流程并导出 map 和 pcd。
- map 与 prior_pcd_file 推荐使用绝对路径并在启动时明确指定。

## 手柄控制

默认启用手柄控制（pb_teleop_twist_joy），按键映射见 qdu2026_nav_bringup/config/* 中的 teleop_twist_joy 配置。

## 其它说明

- 本仓库已去除或不再维护仿真相关的说明与启动流程，若需要仿真支持请参考原项目仓库。
- 部署到实车前请在安全环境下充分测试参数与控制器响应，注意紧急停止与人工接管机制。

欢迎将遇到的问题以 issue 形式提交，或参考原项目 Wiki 获取更多细节,最后再次感谢Shenzhen SMBU PolarBear Robotics Team 的导航开源
