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
source /home/alis/codes/qdu26_sentry_mix/install/setup.bash
source /home/alis/codes/catkin_ws/install/setup.bash
source ~/.bashrc

echo "[1/5] 启动 ros2_libxr..."
ros2 launch ros2_libxr ros2_libxr_launch.py &
LIBXR_PID=$!
sleep 3

echo "[2/5] 清空重置速度命令"
for i in {1..3}; do
    ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
    sleep 0.5
done

echo "[3/5] 启动odin1重定位"
ODIN_LOG=$(mktemp)
ros2 launch odin_ros_driver odin1_ros2.launch.py > >(tee "$ODIN_LOG") 2>&1 &
ODIN_PID=$!

echo "等待odin1重定位完成..."
while ! grep -q "relocalization success" "$ODIN_LOG" 2>/dev/null; do
    sleep 1
done
echo "odin1重定位成功!"

echo "[4/5] 启动 robot_description..."
ros2 launch qdu2026_robot_description robot_description_launch.py &
DESC_PID=$!
sleep 2

echo "[4/5] 启动导航 (world=$WORLD, slam:=False)..."
ros2 launch qdu2026_nav_bringup rm_navigation_launch.py world:=$WORLD slam:=False

# 当导航退出时，清理子进程
kill $LIBXR_PID $DESC_PID $ODIN_PID 2>/dev/null
rm -f "$ODIN_LOG"
