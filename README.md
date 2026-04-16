# DM_IMU_ros2

达妙IMU ROS2驱动

目前仅在ubuntu22.04和ROS2 humble下测试通过

由开源 `https://gitee.com/kit-miao/dm-imu.git` 修改而来

## 功能特性

- **RPY偏移校准** - 支持Roll、Pitch、Yaw三个方向的偏移参数配置
- **IMU校准服务** - 提供`calibrate_imu`服务用于动态校准IMU
- **启动时自动校准** - 支持`calibrate_on_startup`参数在节点启动时自动校准
- **位姿发布** - 支持`publish_pose`参数控制是否发布位姿信息，集成RViz可视化

## 参数配置

在`config/imu_config.yaml`中配置以下参数：

- `calibrate_on_startup` - 启动时是否自动校准 (bool, 默认: false)
- `publish_pose` - 是否发布位姿信息 (bool, 默认: false)  
- `rpy_offset` - RPY偏移值配置 (vector<double>)

## 服务

- `calibrate_imu` - 动态校准IMU传感器

## 快速启动

```bash
# 安装依赖
rosdep install --from-paths src --ignore-src -r -y

# 编译
colcon build

# 运行（带RViz可视化）
ros2 launch dm_imu dm_rviz.launch.py

# 运行（无RViz）
ros2 launch dm_imu run_without_rviz.launch.py

# 调用校准服务
ros2 service call /calibrate_imu dm_imu/srv/CalibrateIMU "{}"
```