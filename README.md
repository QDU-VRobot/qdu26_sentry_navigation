# qdu26_sentry 

整合了实车导航与相关工具包，本包基于 Shenzhen SMBU PolarBear Robotics Team 的导航开源项目 (https://github.com/SMBU-PolarBear-Robotics-Team/qdu2026_sentry_nav)，已移除仿真与无关模块，仅保留并适配实车导航相关功能。感谢原项目的贡献。

支持的 ROS 发行：ROS2 Humble（Ubuntu 22.04 推荐）

qdu2026_sentry_nav主要包
- qdu2026_sentry_nav：实车导航相关代码与启动文件（Nav2 集成、路径跟踪、里程计接口等）。
- qdu2026_nav_bringup：导航启动文件集合（实车启动入口）。
- qdu2026_robot_description：机器人 XMacro 描述、生成 URDF/SDF、发布 robot_description / joint_states。
- sdformat_tools：SDF / XMacro 与 URDF 转换工具（xmacro4sdf、sdf2urdf、UrdfGenerator）。
- rmoss_gz_resources：Gazebo/Ignition 资源库（模型资源，仿真时使用；本仓库保留资源但仿真流程已去除）。
- pcd2pgm：将 PCD 点云转换为 pgm 地图并发布，用于快速生成静态导航地图。
- livox_ros_driver2、point_lio、small_gicp_relocalization、terrain_analysis 等：传感器驱动、里程计、重定位与地形分析模块。

快速开始 — 环境与依赖
1. 系统与 ROS
   - Ubuntu 22.04
   - ROS 2 Humble

2. 必要工具（示例）
   ```bash
   sudo apt update
   sudo apt install -y git python3-pip git-lfs
   pip3 install xmacro
   ```

构建（在工作区根目录）
```bash
# 安装系统依赖
rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y

# 构建（建议保留 --symlink-install 方便调试）
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# 构建完成后
source install/setup.bash
```

使用说明（摘要、实车为主）

1. 启动实车导航（示例）
   - 确认硬件驱动（IMU、里程计、LiDAR、云台）已运行，参数与地图准备就绪。
   - 示例启动命令：
     ```bash
     ros2 launch qdu2026_nav_bringup rm_navigation_launch.py \
       world:=<YOUR_WORLD_NAME> \
       use_robot_state_pub:=False \
     ```
    - 如果只需启动最小导航栈

      ```bash
      ros2 launch qdu2026_nav_bringup rm_navigation_launch.py \
      world:=<YOUR_WORLD_NAME> \
      use_robot_state_pub:=True
      ```

2. qdu2026_robot_description（生成并可视化 URDF）
   - 通过 XMacro 生成 SDF/URDF（推荐在 launch 中生成）：
     Python 示例：
     ```python
     from xmacro.xmacro4sdf import XMLMacro4sdf
     from sdformat_tools.urdf_generator import UrdfGenerator
     xmacro = XMLMacro4sdf()
     xmacro.set_xml_file(robot_xmacro_path)
     xmacro.generate()
     robot_xml = xmacro.to_string()
     urdf_generator = UrdfGenerator()
     urdf_generator.parse_from_sdf_string(robot_xml)
     robot_urdf_xml = urdf_generator.to_string()
     ```
   - 在 RViz 可视化：
     ```bash
     ros2 launch qdu2026_robot_description robot_description_launch.py
     ```

3. sdformat_tools（工具使用）
   - 安装 xmacro（已示）后可以直接使用命令行工具：
     ```bash
     # 将 xmacro -> sdf
     xmacro4sdf model.sdf.xmacro > model.sdf

     # sdf -> urdf（有限制）
     sdf2urdf model.sdf
     ```

4. pcd2pgm（点云转地图）
   - 启动节点并查看 RViz：
     ```bash
     ros2 launch pcd2pgm pcd2pgm_launch.py
     ```
   - 保存生成的栅格地图：
     ```bash
     ros2 run nav2_map_server map_saver_cli -f <YOUR_MAP_NAME>
     ```
   - 参数文件在 pcd2pgm/pcd2pgm.yaml，可调整滤波与分辨率等参数。

5. rmoss_gz_resources
   - 包含多种 Gazebo/Ignition 模型与 xmacro 模块（如裁判系统、标准机器人）。注意：本仓库主要面向实车，仿真流程已删除；如需仿真请参考原项目与 Gazebo/Ignition 使用说明。

常用启动参数说明（简要）
- use_sim_time: 实车设 False
- slam: 建图模式（实车一般 False）
- map / prior_pcd_file: 推荐使用绝对路径
- params_file: Nav2 与各节点参数
- autostart: 是否自动启动 Nav2
- use_robot_state_pub: 是否由本包发布 robot_state（实车常由硬件独立模块提供）

安全与调试提示
- 部署到实车前在受控环境充分测试参数与控制器响应。
- 准备好紧急停止与人工接管机制。
- 若发生 TF 丢失或话题异常，先检查各驱动节点与 remapping、frame_id 配置。

参考与来源
- 原项目：https://github.com/SMBU-PolarBear-Robotics-Team/qdu2026_sentry_nav
- sdformat_tools / xmacro：用于 XMacro 与 SDF/URDF 转换的工具
- rmoss_gz_resources：Gazebo / Ignition 模型资源库（仿真参考）



欢迎将遇到的问题以 issue 形式提交，或参考原项目 Wiki 获取更多细节,最后再次感谢Shenzhen SMBU PolarBear Robotics Team 的导航开源