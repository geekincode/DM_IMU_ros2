#!/bin/bash

# 校准 DM IMU 的姿态，将 RPY 都重置为 0

echo "========================================="
echo "DM IMU 姿态校准工具"
echo "========================================="
echo ""

# 检查 ROS2 环境
if [ -z "$ROS_DOMAIN_ID" ]; then
    echo "警告: ROS_DOMAIN_ID 未设置"
fi

echo "调用校准服务..."
ros2 service call /calibrate_imu dm_imu/srv/CalibrateIMU

if [ $? -eq 0 ]; then
    echo ""
    echo "✓ 校准成功！"
    echo "IMU 的 RPY 已重置为 0"
    echo ""
    echo "您现在可以通过以下命令查看校准结果:"
    echo "  ros2 topic echo /imu/data"
else
    echo ""
    echo "✗ 校准失败！"
    echo "请确保 dm_imu_node 正在运行"
    exit 1
fi
