#!/bin/bash
# 累积合并多帧点云，Ctrl+C 后降采样保存为 PCD
# 用法: ./save_pcd.sh [输出文件名，默认 map.pcd] [体素大小，默认 0.05m]

OUTPUT=${1:-map.pcd}
VOXEL=${2:-0.05}

python3 - <<EOF
import rclpy, sys, signal
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np

all_pts = []

def voxel_downsample(pts, voxel_size):
    indices = np.floor(pts / voxel_size).astype(np.int32)
    _, unique = np.unique(indices, axis=0, return_index=True)
    return pts[unique]

def save_and_exit(*_):
    if not all_pts:
        print('未收到任何数据')
        sys.exit(1)
    pts = np.concatenate(all_pts).astype(np.float64)
    pts = voxel_downsample(pts, ${VOXEL})
    with open('${OUTPUT}', 'w') as f:
        f.write(f'# .PCD v0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n')
        f.write(f'WIDTH {len(pts)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n')
        f.write(f'POINTS {len(pts)}\nDATA ascii\n')
        for p in pts:
            f.write(f'{p[0]:.6f} {p[1]:.6f} {p[2]:.6f}\n')
    print(f'\n已保存 ${OUTPUT} ({len(pts)} 个点，体素=${VOXEL}m，共 {len(all_pts)} 帧)')
    sys.exit(0)

signal.signal(signal.SIGINT, save_and_exit)

class Saver(Node):
    def __init__(self):
        super().__init__('pcd_saver')
        self.create_subscription(PointCloud2, '/odin1/cloud_slam', self.cb, 10)
        self.get_logger().info(f'累积中，Ctrl+C 停止并保存（体素=${VOXEL}m）...')

    def cb(self, msg):
        pts = np.array([[p[0],p[1],p[2]] for p in pc2.read_points(msg, field_names=('x','y','z'), skip_nans=True)], dtype=np.float64)
        if len(pts):
            all_pts.append(pts)
            print(f'\r已累积 {len(all_pts)} 帧', end='', flush=True)

rclpy.init()
rclpy.spin(Saver())
EOF
