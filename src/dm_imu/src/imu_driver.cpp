#include "dm_imu/imu_driver.h"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <chrono>
#include <thread>
#include <tf2_ros/transform_broadcaster.h>


namespace dmbot_serial {

// 构造函数实现
DmImu::DmImu(rclcpp::Node::SharedPtr node)
    : node_(node), imu_msgs(std::make_unique<sensor_msgs::msg::Imu>())
{
    // 获取参数
    node->declare_parameter("port", "/dev/ttyACM0");
    node->declare_parameter("baud", 921600);
    
    imu_serial_port = node->get_parameter("port").as_string();
    imu_seial_baud = node->get_parameter("baud").as_int();

    imu_msgs->header.frame_id = "imu_link";

    init_imu_serial();

    // 初始化发布器
    imu_pub_ = node->create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);
    imu_pose_pub_ = node->create_publisher<geometry_msgs::msg::PoseStamped>("pose", 10);
    euler2_pub_ = node->create_publisher<std_msgs::msg::Float64MultiArray>("dm/imu", 9);

    transform_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node);

    // 进入设置模式
    enter_setting_mode();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    // 配置传感器
    turn_on_accel();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    turn_on_gyro();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    turn_on_euler();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    turn_off_quat();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    set_output_1000HZ();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    save_imu_para();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    exit_setting_mode();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    // 启动数据读取线程
    rec_thread = std::thread(&DmImu::get_imu_data_thread, this);

    RCLCPP_INFO(node_->get_logger(), "IMU Initialization Complete");
}

// 析构函数实现
DmImu::~DmImu()
{
    RCLCPP_DEBUG(node_->get_logger(), "Entering ~DmImu()");
    stop_thread_ = true;
    if (rec_thread.joinable()) {
        rec_thread.join();
    }
    if (serial_imu.isOpen()) {
        serial_imu.close();
    }
}

// 初始化串口实现
void DmImu::init_imu_serial()
{
    try {
        serial_imu.setPort(imu_serial_port);
        serial_imu.setBaudrate(imu_seial_baud);
        serial_imu.setFlowcontrol(serial::flowcontrol_none);
        serial_imu.setParity(serial::parity_none);
        serial_imu.setStopbits(serial::stopbits_one);
        serial_imu.setBytesize(serial::eightbits);
        serial::Timeout timeout = serial::Timeout::simpleTimeout(20);
        serial_imu.setTimeout(timeout);
        serial_imu.open();

    } 
    catch (const serial::IOException& e) {
        RCLCPP_FATAL(node_->get_logger(), "In initialization,Unable to open imu serial port: %s", e.what());
        exit(0);
    }

    if (serial_imu.isOpen()) {
        RCLCPP_INFO(node_->get_logger(), "In initialization,Imu Serial Port initialized: %s @ %d baud",
                   imu_serial_port.c_str(), imu_seial_baud);
    } else {
        RCLCPP_FATAL(node_->get_logger(), "In initialization,Unable to open imu serial port ");
        exit(0);
    }
}

// 数据读取线程实现
void DmImu::get_imu_data_thread()
{
    RCLCPP_INFO(node_->get_logger(), "In get_imu_data_thread,Imu Serial Port initialized: %s @ %d baud",
                   imu_serial_port.c_str(), imu_seial_baud);
    int error_num = 0;

    while (rclcpp::ok() && !stop_thread_)
    {
        // 检查串口是否打开
        if (!serial_imu.isOpen()) {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, 
                                "In get_imu_data_thread,imu serial port unopen");
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        try {
            serial_imu.read((uint8_t*)(&receive_data.FrameHeader1),4);

            if(receive_data.FrameHeader1==0x55&&receive_data.flag1==0xAA&&receive_data.slave_id1==0x01&&receive_data.reg_acc==0x01)
            {
            RCLCPP_DEBUG(node_->get_logger(),"包头正确");

            serial_imu.read((uint8_t*)(&receive_data.accx_u32),57-4); 

            #if IMU_SERIAL_DEBUG == 1
            // 打印结构体所有字段
            RCLCPP_INFO_STREAM(node_->get_logger(), 
                "IMU Receive Frame Data:" << std::endl
                << "FrameHeader1: 0x" << std::hex << static_cast<int>(receive_data.FrameHeader1) << std::dec << std::endl
                << "flag1: 0x" << std::hex << static_cast<int>(receive_data.flag1) << std::dec << std::endl
                << "slave_id1: 0x" << std::hex << static_cast<int>(receive_data.slave_id1) << std::dec << std::endl
                << "reg_acc: 0x" << std::hex << static_cast<int>(receive_data.reg_acc) << std::dec << std::endl
                << "accx_u32: " << receive_data.accx_u32 << std::endl
                << "accy_u32: " << receive_data.accy_u32 << std::endl
                << "accz_u32: " << receive_data.accz_u32 << std::endl
                << "crc1: 0x" << std::hex << receive_data.crc1 << std::dec << std::endl
                << "FrameEnd1: 0x" << std::hex << static_cast<int>(receive_data.FrameEnd1) << std::dec << std::endl

                << "FrameHeader2: 0x" << std::hex << static_cast<int>(receive_data.FrameHeader2) << std::dec << std::endl
                << "flag2: 0x" << std::hex << static_cast<int>(receive_data.flag2) << std::dec << std::endl
                << "slave_id2: 0x" << std::hex << static_cast<int>(receive_data.slave_id2) << std::dec << std::endl
                << "reg_gyro: 0x" << std::hex << static_cast<int>(receive_data.reg_gyro) << std::dec << std::endl
                << "gyrox_u32: " << receive_data.gyrox_u32 << std::endl
                << "gyroy_u32: " << receive_data.gyroy_u32 << std::endl
                << "gyroz_u32: " << receive_data.gyroz_u32 << std::endl
                << "crc2: 0x" << std::hex << receive_data.crc2 << std::dec << std::endl
                << "FrameEnd2: 0x" << std::hex << static_cast<int>(receive_data.FrameEnd2) << std::dec << std::endl

                << "FrameHeader3: 0x" << std::hex << static_cast<int>(receive_data.FrameHeader3) << std::dec << std::endl
                << "flag3: 0x" << std::hex << static_cast<int>(receive_data.flag3) << std::dec << std::endl
                << "slave_id3: 0x" << std::hex << static_cast<int>(receive_data.slave_id3) << std::dec << std::endl
                << "reg_euler: 0x" << std::hex << static_cast<int>(receive_data.reg_euler) << std::dec << std::endl
                << "roll_u32: " << receive_data.roll_u32 << std::endl
                << "pitch_u32: " << receive_data.pitch_u32 << std::endl
                << "yaw_u32: " << receive_data.yaw_u32 << std::endl
                << "crc3: 0x" << std::hex << receive_data.crc3 << std::dec << std::endl
                << "FrameEnd3: 0x" << std::hex << static_cast<int>(receive_data.FrameEnd3) << std::dec);
            #endif

            // CRC校验
            if (Get_CRC16((uint8_t*)(&receive_data.FrameHeader1), 16)==receive_data.crc1) {
              RCLCPP_DEBUG(node_->get_logger(),"CRC校验正确");
                // std::cerr<<"calculate1: "<<Get_CRC16((uint8_t*)(&receive_data.FrameHeader1), 16)<<std::endl;
                // std::cerr<<"actuaal1: "<<receive_data.crc1<<std::endl;
                data.accx =*((float *)(&receive_data.accx_u32));
                data.accy =*((float *)(&receive_data.accy_u32));
                data.accz =*((float *)(&receive_data.accz_u32));
                // RCLCPP_WARN(node_->get_logger(), "Accelerometer CRC error");
                // crc_valid = false;
            }
            if (Get_CRC16((uint8_t*)(&receive_data.FrameHeader2), 16)==receive_data.crc2) {
                // std::cerr<<"calculate2: "<<Get_CRC16((uint8_t*)(&receive_data.FrameHeader2), 16)<<std::endl;
                // std::cerr<<"actuaal2: "<<receive_data.crc2<<std::endl;
                data.gyrox =*((float *)(&receive_data.gyrox_u32));
                data.gyroy =*((float *)(&receive_data.gyroy_u32));
                data.gyroz =*((float *)(&receive_data.gyroz_u32));
            }
            if (Get_CRC16((uint8_t*)(&receive_data.FrameHeader3), 16)==receive_data.crc3) {
                // std::cerr<<"calculate3: "<<Get_CRC16((uint8_t*)(&receive_data.FrameHeader3), 16)<<std::endl;
                // std::cerr<<"actuaal3: "<<receive_data.crc3<<std::endl;
                data.roll =*((float *)(&receive_data.roll_u32));
                data.pitch =*((float *)(&receive_data.pitch_u32));
                data.yaw =*((float *)(&receive_data.yaw_u32));
            }

            publish_imu_data();
            }

        } catch (const serial::IOException& e) {
            RCLCPP_ERROR(node_->get_logger(), "Serial IO error: %s", e.what());
            init_imu_serial(); // 尝试重新初始化
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), "Processing error: %s", e.what());
        }
    }
}


void DmImu::publish_imu_data()
{
    std::lock_guard<std::mutex> lock(data_mutex_);

    RCLCPP_DEBUG(node_->get_logger(), "Publishing IMU data...");
    auto now = node_->get_clock()->now();

    auto imu_msg = std::make_unique<sensor_msgs::msg::Imu>();
    if (!imu_msg) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to allocate imu_msg");
        return;
    }

    imu_msg->header.stamp = now;
    imu_msg->header.frame_id = "imu_link";

    tf2::Quaternion q;
    try {
        q.setRPY(
            data.roll * M_PI / 180.0,
            data.pitch * M_PI / 180.0,
            data.yaw * M_PI / 180.0
        );
        RCLCPP_DEBUG(node_->get_logger(), "转换欧拉角到四元数");
    } catch (const tf2::InvalidArgumentException& e) {
        RCLCPP_ERROR(node_->get_logger(), "Invalid Euler angles: roll=%.2f, pitch=%.2f, yaw=%.2f", 
                    data.roll, data.pitch, data.yaw);
        return;
    }

    imu_msg->orientation = tf2::toMsg(q);

    imu_msg->angular_velocity.x = data.gyrox;
    imu_msg->angular_velocity.y = data.gyroy;
    imu_msg->angular_velocity.z = data.gyroz;

    imu_msg->linear_acceleration.x = data.accx;
    imu_msg->linear_acceleration.y = data.accy;
    imu_msg->linear_acceleration.z = data.accz;

    auto header = imu_msg->header;  // 保存 header 副本
    auto orientation = imu_msg->orientation;
    imu_pub_->publish(std::move(imu_msg));  // imu_msg 被 move 后不能再使用

    // 发布 PoseStamped
    if (imu_pose_pub_) {
        auto pose_msg = std::make_unique<geometry_msgs::msg::PoseStamped>();
        if (!pose_msg) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to allocate pose_msg");
            return;
        }
        pose_msg->header = header;  // 使用 header 副本
        pose_msg->pose.orientation = orientation;  // 注意：imu_msg 已经被 move，不能使用！
        pose_msg->pose.position.x = 0.0;
        pose_msg->pose.position.y = 0.0;
        pose_msg->pose.position.z = 0.0;
        imu_pose_pub_->publish(std::move(pose_msg));
    } else {
        RCLCPP_ERROR(node_->get_logger(), "imu_pose_pub_ is null");
    }

    geometry_msgs::msg::TransformStamped tfs;
    tfs.header.stamp = now;
    tfs.header.frame_id = "base_link";      // 父坐标系
    tfs.child_frame_id = "imu_link";        // 子坐标系
    tfs.transform.rotation = orientation;

    // 如果 IMU 安装位置有偏移，可在这里添加平移
    tfs.transform.translation.x = 0.0;
    tfs.transform.translation.y = 0.0;
    tfs.transform.translation.z = 0.0;

    if (transform_broadcaster_) {
        transform_broadcaster_->sendTransform(tfs);
    } else {
        RCLCPP_WARN(node_->get_logger(), "Transform broadcaster not initialized.");
    }

    RCLCPP_DEBUG(node_->get_logger(), "发布IMU 数据成功");
}

// 以下是私有方法实现
void DmImu::enter_setting_mode()
{
    // 发送进入设置模式的命令
    const uint8_t txbuf[4] = {0xAA, 0x06, 0x01, 0x0D};
    try {
        for(int i=0;i<5;i++){
          if (serial_imu.isOpen()) {
              serial_imu.write(txbuf, sizeof(txbuf));
          }
          rclcpp::sleep_for(std::chrono::milliseconds(10));
        }
    } catch (const serial::IOException& e) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to enter setting mode: %s", e.what());
    }
}

void DmImu::turn_on_accel()
{
    // 发送开启加速度计的命令
    const uint8_t txbuf[4] = {0xAA, 0x01, 0x14, 0x0D};
    for(int i=0;i<5;i++)
    {
      serial_imu.write(txbuf,sizeof(txbuf));
      rclcpp::sleep_for(std::chrono::milliseconds(10));
    }
}

void DmImu::turn_on_gyro()
{
    // 发送开启陀螺仪的命令
    const uint8_t txbuf[] = {0xAA, 0x01, 0x15, 0x0D};
    for(int i=0;i<5;i++)
    {
      serial_imu.write(txbuf,sizeof(txbuf));
      rclcpp::sleep_for(std::chrono::milliseconds(10));
    }
}

void DmImu::turn_on_euler()
{
    // 发送开启欧拉角的命令
    const uint8_t txbuf[4] = {0xAA, 0x01, 0x16, 0x0D};
    for(int i=0;i<5;i++)
    {
      serial_imu.write(txbuf,sizeof(txbuf));
      rclcpp::sleep_for(std::chrono::milliseconds(10));
    }
}

void DmImu::turn_off_quat()
{
    // 发送关闭四元数的命令
  const uint8_t txbuf[4]={0xAA,0x01,0x07,0x0D};
  for(int i=0;i<5;i++)
  {
    serial_imu.write(txbuf,sizeof(txbuf));
    rclcpp::sleep_for(std::chrono::milliseconds(10));
  }
}

void DmImu::set_output_1000HZ()
{
    // 设置输出频率为1000Hz
  const uint8_t txbuf[5]={0xAA, 0x02, 0x01, 0x00, 0x0D};
  for(int i=0;i<5;i++)
  {
    serial_imu.write(txbuf,sizeof(txbuf));
    rclcpp::sleep_for(std::chrono::milliseconds(10));
  }
}

void DmImu::save_imu_para()
{
    // 保存参数
  const uint8_t txbuf[4]={0xAA, 0x03, 0x01, 0x0D};
  for(int i=0;i<5;i++)
  {
    serial_imu.write(txbuf,sizeof(txbuf));
    rclcpp::sleep_for(std::chrono::milliseconds(10));
  }
}

void DmImu::exit_setting_mode()
{
    // 退出设置模式
  const uint8_t txbuf[4]={0xAA, 0x06, 0x00, 0x0D};
  for(int i=0;i<5;i++)
  {
    serial_imu.write(txbuf,sizeof(txbuf));
    rclcpp::sleep_for(std::chrono::milliseconds(10));
  }
}

void DmImu::restart_imu()
{
    // 重启IMU
  const uint8_t txbuf[4]={0xAA, 0x00, 0x00, 0x0D};
  for(int i=0;i<5;i++)
  {
    serial_imu.write(txbuf,sizeof(txbuf));
    rclcpp::sleep_for(std::chrono::milliseconds(10));
  }
}

} // namespace dmbot_serial