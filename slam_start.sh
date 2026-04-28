#!/bin/bash
# 脚本1: SLAM建图启动脚本

source /opt/ros/humble/setup.bash
source /home/alis/codes/ros2_libxr/install/setup.bash
source /home/alis/codes/qdu26_sentry/install/setup.bash
source ~/.bashrc
 
echo "[1/3] 启动 ros2_libxr..."
ros2 launch ros2_libxr ros2_libxr_launch.py &
LIBXR_PID=$!
sleep 3

for i in {1..3}; do
    ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
    sleep 0.5
done

echo "[2/3] 启动 robot_description..."
ros2 launch qdu2026_robot_description robot_description_launch.py &
DESC_PID=$!
sleep 2

echo "[3/3] 启动导航 (SLAM模式)..."
ros2 launch qdu2026_nav_bringup rm_navigation_launch.py slam:=True

# 当导航退出时，清理子进程
kill $LIBXR_PID $DESC_PID 2>/dev/null
