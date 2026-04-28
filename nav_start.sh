#!/bin/bash
# 脚本2: 导航启动脚本 (需要指定地图world参数)

if [ -z "$1" ]; then
    echo "错误: 必须指定 world 参数"
    echo "用法: $0 <world名称>"
    echo "示例: $0 306"
    exit 1
fi

WORLD=$1

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

echo "[3/3] 启动导航 (world=$WORLD, slam:=False)..."
ros2 launch qdu2026_nav_bringup rm_navigation_launch.py world:=$WORLD slam:=False

# 当导航退出时，清理子进程
kill $LIBXR_PID $DESC_PID 2>/dev/null
