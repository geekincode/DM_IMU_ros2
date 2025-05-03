#include "rclcpp/rclcpp.hpp"
#include "dm_imu/imu_driver.h"
#include <iostream>
#include <thread>
#include <condition_variable>
#include <chrono>

int main(int argc, char **argv)
{
    try {
        // 初始化 ROS 2 节点
        rclcpp::init(argc, argv);
        auto node = rclcpp::Node::make_shared("dm_imu_node");

        // 创建 DmImu 对象
        auto imuInterface = std::make_shared<dmbot_serial::DmImu>(node);

        // 主循环控制
        auto loop_rate = std::chrono::milliseconds(1); // 1 ms

        // 主循环
        while (rclcpp::ok()) {
            auto start = node->get_clock()->now();

            rclcpp::spin_some(node);

            // 计算剩余睡眠时间
            auto end = node->get_clock()->now();
            auto elapsed = (end - start).nanoseconds();
            auto sleep_time_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(loop_rate).count() - elapsed;

            if (sleep_time_ns > 0) {
                std::this_thread::sleep_for(std::chrono::nanoseconds(sleep_time_ns));
            }
        }

        // 显式清理资源
        imuInterface.reset(); // 确保 DmImu 对象析构

        // 关闭节点
        rclcpp::shutdown();

    } catch (const std::exception& e) {
        std::cerr << "Exception in main: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}