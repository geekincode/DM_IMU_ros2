#ifndef _IMU_DRIVER_H_
#define _IMU_DRIVER_H_
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <initializer_list>
#include <fstream>
#include <array>
#include <serial/serial.h> // 确保安装了serial库
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include "dm_imu/bsp_crc.h"
#include "std_msgs/msg/float64_multi_array.hpp"

#define IMU_SERIAL_DEBUG 0      //{0:none, 1:all_Rxdata}

namespace dmbot_serial
{
#pragma pack(1)
  typedef struct
  {
    uint8_t FrameHeader1;                   //帧头
    uint8_t flag1;                          //标志号
    uint8_t slave_id1;                      //从机ID，指令设置的 CANID
    uint8_t reg_acc;                        //寄存器地址
    uint32_t accx_u32;                      //X轴加速度
    uint32_t accy_u32;                      //Y轴加速度
    uint32_t accz_u32;                      //Z轴加速度
    uint16_t crc1;                          //CRC16校验位
    uint8_t FrameEnd1;                      //帧尾

    uint8_t FrameHeader2;                   //帧头
    uint8_t flag2;                          //标志号
    uint8_t slave_id2;                      //从机ID
    uint8_t reg_gyro;                       //寄存器ID
    uint32_t gyrox_u32;                     //X轴角速度
    uint32_t gyroy_u32;                     //Y轴角速度
    uint32_t gyroz_u32;                     //Z轴角速度
    uint16_t crc2;                          //CRC16校验位
    uint8_t FrameEnd2;                      //帧尾

    uint8_t FrameHeader3;                   //帧头
    uint8_t flag3;                          //标志位
    uint8_t slave_id3;                      //从机ID
    uint8_t reg_euler;//r-p-y               //寄存器ID
    uint32_t roll_u32;                      //Roll横滚角
    uint32_t pitch_u32;                     //Pitch俯仰角
    uint32_t yaw_u32;                       //Yaw偏航角
    uint16_t crc3;                          //CRC16校验位
    uint8_t FrameEnd3;                      //帧尾
  }IMU_Receive_Frame;
#pragma pack()    //角速度单位为 rad/s，加速度单位为 m/s^2 ，角度单位为°

  typedef struct
  {
    float accx;
    float accy;
    float accz;
    float gyrox;
    float gyroy;
    float gyroz;
    float roll;
    float pitch;
    float yaw;
  }IMU_Data;

  class DmImu 
  {
  public:
      // 构造函数和析构函数声明
      DmImu(rclcpp::Node::SharedPtr node);
      ~DmImu();

      // 公有方法声明
      void init_imu_serial();
      void get_imu_data_thread();
      void publish_imu_data();

  private:
      // 私有成员变量声明
      rclcpp::Node::SharedPtr node_;
      std::shared_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;
      int imu_seial_baud;
      std::string imu_serial_port;
      serial::Serial serial_imu;
      std::thread rec_thread;
      rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
      rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr imu_pose_pub_;
      rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr euler2_pub_;

      std::unique_ptr<sensor_msgs::msg::Imu> imu_msgs;
      std::atomic<bool> stop_thread_{false};
      IMU_Receive_Frame receive_data{};
      std::mutex data_mutex_;
      IMU_Data data{};

    // 私有方法声明
    void enter_setting_mode();
    void turn_on_accel();
    void turn_on_gyro();
    void turn_on_euler();
    void turn_off_quat();
    void set_output_1000HZ();
    void save_imu_para();
    void exit_setting_mode();
    void restart_imu();
};

}
#endif