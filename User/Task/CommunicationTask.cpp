#include "../Task/CommunicationTask.hpp"
#include "../Task/ShootTask.hpp"

#include "../APP/Mod/RemoteModeManager.hpp"
#include "../APP/Tools.hpp"
#include "../BSP/IMU/HI12H3_IMU.hpp"
#include "../BSP/Motor/DM/DmMotor.hpp"
#include "../BSP/Motor/Dji/DjiMotor.hpp"
#include "../BSP/Remote/Dbus/Dbus.hpp"
#include "../HAL/CAN/can_hal.hpp"

#include "../APP/Mod/RemoteModeManager.hpp"

#include "usbd_cdc_if.h"
// #include "usb_device.h"
#include "cmsis_os2.h"
#include "tim.h"
#include "usart.h"
#include <cstring>

#define SIZE 8
uint8_t format[15];
uint64_t i;
    uint8_t Rx_pData[19];
Communicat::Gimbal_to_Chassis Gimbal_to_Chassis_Data;

// Debug variables for watching in debugger
Communicat::Vision::Rx_Target debug_rx_target;
Communicat::Vision::Rx_Other debug_rx_other;

// delay PID测试
float time_kp = 2.0, time_out, int_time = 5;
uint32_t demo_time; // 测试时间戳

void CommunicationTask(void *argument)
{
    for (;;)
    {
        Communicat::vision.Data_send();
        Communicat::vision.dataReceive();
        Gimbal_to_Chassis_Data.Data_send();
        // Gimbal_to_Chassis_Data.Receive();

        osDelay(4);
    }
}

namespace Communicat
{

void Vision::time_demo()
{
    if (demo_time > 20 || demo_time < 200)
    {
        time_out = time_kp * demo_time;
        int_time = (uint32_t)(time_out);

        if (int_time < 5)
        {
            int_time = 5;
        }
        else if (int_time > 15)
        {
            int_time = 15;
        }
    }
}

void Gimbal_to_Chassis::Data_send()
{
    using namespace BSP;
    auto *remote = Mode::RemoteModeManager::Instance().getActiveController();
    auto channel_to_uint8 = [](float value) { return (static_cast<uint8_t>(value * 110) + 110); };

    // 初始化结构体数据
    if (remote->isKeyboardMode() == true)
    {
        direction.LX = channel_to_uint8(-direction.LX);
        direction.LY = channel_to_uint8(-direction.LY);
    }
    else
    {
        direction.LX = channel_to_uint8(BSP::Remote::dr16.remoteLeft().x);
        direction.LY = channel_to_uint8(BSP::Remote::dr16.remoteLeft().y);
    }

    direction.Yaw_encoder_angle_err = CalcuGimbalToChassisAngle();

    chassis_mode.Universal_mode = remote->isUniversalMode();
    chassis_mode.Follow_mode = remote->isFollowMode();
    chassis_mode.Rotating_mode = remote->isRotatingMode();
    chassis_mode.KeyBoard_mode = remote->isKeyboardMode();

    chassis_mode.stop = remote->isStopMode();
    ui_list.friction_enabled = TASK::Shoot::shoot_fsm.getFrictionState();

    if (chassis_mode.Rotating_mode)
        direction.Rotating_vel = channel_to_uint8(BSP::Remote::dr16.sw());

    auto key = BSP::Remote::dr16.keyBoard();
    ui_list.UI_F5 = key.ctrl;

    if (demo_time > 200)
    {
        ui_list.Vision = 0;
    }


    ui_list.aim_x = vision.getAimX();
    ui_list.aim_y = vision.getAimY();
    ui_list.projectile_count = TASK::Shoot::shoot_fsm.getProjectileCount();

    // 计算总数据长度 - 两帧发送
    // Frame1: Head(1) + Direction前7字节 = 8字节
    // Frame2: Direction后2字节 + ChassisMode(1) + UiList(5) = 8字节

    // 使用临时指针将数据拷贝到缓冲区
    len = sizeof(direction) + sizeof(chassis_mode) + sizeof(ui_list) + 1;
    uint8_t tx_data[16];
    auto temp_ptr = tx_data;

    *temp_ptr = head;
    temp_ptr++;

    const auto memcpy_safe = [&](const auto &data) {
        std::memcpy(temp_ptr, &data, sizeof(data));
        temp_ptr += sizeof(data);
    };

    memcpy_safe(direction);    // 序列化方向数据
    memcpy_safe(chassis_mode); // 序列化模式数据
    memcpy_safe(ui_list);      // 序列化UI状态

    // 获取CAN1设备实例 
    auto& can2 = HAL::CAN::get_can_bus_instance().get_device(HAL::CAN::CanDeviceId::HAL_Can2);
    if (HAL_CAN_GetTxMailboxesFreeLevel(can2.get_handle()) == 0)
    {
        return;  // 邮箱满，跳过本次发送
    }

    // 分两帧通过CAN发送
    HAL::CAN::Frame frame1;
    frame1.id = CAN_G2C_FRAME1_ID;
    frame1.dlc = 8;
    frame1.is_extended_id = false;
    frame1.is_remote_frame = false;
    
    // 第一帧: Head + Direction前7字节 (0-7)
    std::memcpy(can_tx_buffer[0], tx_data, 8);
    std::memcpy(frame1.data, can_tx_buffer[0], 8);
    can2.send(frame1);
    
    HAL::CAN::Frame frame2;
    frame2.id = CAN_G2C_FRAME2_ID;
    frame2.dlc = 8;
    frame2.is_extended_id = false;
    frame2.is_remote_frame = false;
    
    std::memcpy(can_tx_buffer[1], tx_data + 8, 8);  // 剩余8字节数据

    std::memcpy(frame2.data, can_tx_buffer[1], 8);
    can2.send(frame2);
}

void Gimbal_to_Chassis::HandleCANMessage(uint32_t std_id, uint8_t* data)
{
    // 单帧接收 - 双字节帧头校验 (0x21, 0x12)
    
    // 验证帧头
    if (data[0] != 0x21 || data[1] != 0x12) {
        return;
    }

    // 更新接收时间戳
    last_frame_time = HAL_GetTick();
    
    // 帧头校验成功，直接解析RxRefree结构体数据
    std::memcpy(&rx_refree, data, sizeof(rx_refree));
}
float Gimbal_to_Chassis::CalcuGimbalToChassisAngle()
{

    float encoder_angle = BSP::Motor::DM::Motor4310.getAngle0_360(2, 1.0f);

    // 计算最终角度误差 --------------------------------------------------
    return Tools.Zero_crossing_processing(Init_Angle, encoder_angle, 360.0f) - encoder_angle;
}

float rx_angle;
double ins_dt;
uint32_t INS_DWT_Count = 0;

uint32_t send_time;
uint16_t demo_angle = 4050;

void Vision::Data_send()
{
    frame.head_one = 0x39;
    frame.head_two = 0x39;

    tx_gimbal.yaw_angle = BSP::IMU::imu.getAddYaw() * 100;
    tx_gimbal.pitch_angle = BSP::IMU::imu.getPitch() * 100;

    tx_other.bullet_rate = 26;
    tx_other.enemy_color = 0x52; // 0x42我红   0X52我蓝

    tx_other.tail = 0xFF; // 准备标志位

    Tx_pData[0] = frame.head_one;
    Tx_pData[1] = frame.head_two;

    Tx_pData[2] = (int32_t)tx_gimbal.pitch_angle >> 24;
    Tx_pData[3] = (int32_t)tx_gimbal.pitch_angle >> 16;
    Tx_pData[4] = (int32_t)tx_gimbal.pitch_angle >> 8;
    Tx_pData[5] = (int32_t)tx_gimbal.pitch_angle;

    Tx_pData[6] = (int32_t)tx_gimbal.yaw_angle >> 24;
    Tx_pData[7] = (int32_t)tx_gimbal.yaw_angle >> 16;
    Tx_pData[8] = (int32_t)tx_gimbal.yaw_angle >> 8;
    Tx_pData[9] = (int32_t)tx_gimbal.yaw_angle;

    Tx_pData[10] = 26;
    Tx_pData[11] = 0x52; // 0x42红   0X52蓝色
    Tx_pData[12] = tx_other.vision_mode;
    Tx_pData[13] = tx_other.tail;

    send_time++;
    tx_gimbal.time = send_time;
    rx_target.time = (Rx_pData[13] << 24 | Rx_pData[14] << 16 | Rx_pData[15] << 8 | Rx_pData[16]);

    demo_time = tx_gimbal.time - rx_target.time;

    Tx_pData[14] = (int32_t)tx_gimbal.time >> 24;
    Tx_pData[15] = (int32_t)tx_gimbal.time >> 16;
    Tx_pData[16] = (int32_t)tx_gimbal.time >> 8;
    Tx_pData[17] = (int32_t)tx_gimbal.time;

    CDC_Transmit_FS(Tx_pData, 18);
}

void Vision::dataReceive()
{
    //static uint32_t last_vision_update_tick = 0; 

    uint32_t rx_len = 19;

    CDC_Receive_FS(Rx_pData, &rx_len);

    if (Rx_pData[0] == 0x39 && Rx_pData[1] == 0x39)
    {
        rx_target.pitch_angle = (Rx_pData[2] << 24 | Rx_pData[3] << 16 | Rx_pData[4] << 8 | Rx_pData[5]) / 100.0;
        rx_target.yaw_angle = (Rx_pData[6] << 24 | Rx_pData[7] << 16 | Rx_pData[8] << 8 | Rx_pData[9]) / 100.0;
        rx_other.vision_ready = Rx_pData[10];
	
        if (rx_other.vision_ready == false || rx_other.vision_ready == 0)
        {
            vision_flag = false;
        }
        else
        {
            vision_flag = true;
        }

		//如果视觉时间戳差值大于100ms，判断为断连
//		if(vision_flag == true && send_time - rx_target.time > 100)
//		{
//			vision_flag = false;
//		}
		
        //		if((rx_other.vision_ready == false))    
        //			vision_flag = false;
        //		else
        //			vision_flag = true;

        yaw_angle_ = rx_target.yaw_angle - BSP::Motor::DM::Motor4310.getAngleDeg(2);
        pitch_angle_ = rx_target.pitch_angle + BSP::Motor::DM::Motor4310.getAngleDeg(1);
        //pitch_angle_ *= -1.0; // 每台方向不同
        yaw_angle_ *= -1.0;

        rx_other.fire = (Rx_pData[11]);
        rx_other.tail = Rx_pData[12];
        rx_other.aim_x = Rx_pData[17];
        rx_other.aim_y = Rx_pData[18];

        // Update debug variables
        debug_rx_target = rx_target;
        debug_rx_other = rx_other;
    }
}
}; // namespace Communicat