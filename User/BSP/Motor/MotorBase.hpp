
#pragma once

#include "../CAN/Bsp_Can.hpp"
#include "../StaticTime.hpp"
#include "../../HAL/CAN/can_hal.hpp"
namespace BSP::Motor
{
template <uint8_t N> class MotorBase
{
  protected:
    struct UnitData
    {
        double angle_Deg; // 单位度角度
        double angle_Rad; // 单位弧度角度

        double velocity_Rad; // 单位弧度
        double velocity_Rpm; // 单位rpm

        double current_A;     // 单位安培
        double torque_Nm;     // 单位牛米
        double temperature_C; // 单位摄氏度

        double last_angle;
        double add_angle;
    };

    struct RunTime
    {
        RM_StaticTime dirTime; // 运行时间
        bool Dir_Flag;
    };

    UnitData unit_data_[N]; // 国际单位数据
    RunTime runTime_[N];

    virtual void Parse(const CAN_RxHeaderTypeDef RxHeader, const uint8_t *pData) = 0;
      protected:
    /**
     * @brief 发送CAN帧的通用方法
     * 
     * @param can_id CAN ID
     * @param data 要发送的数据
     * @param dlc 数据长度
     * @param mailbox 邮箱编号
     */


  public:
      void send_can_frame(uint32_t can_id, const uint8_t* data, uint8_t dlc, uint32_t mailbox = CAN_TX_MAILBOX1)
    {
        auto& can_bus = HAL::CAN::get_can_bus_instance();
        HAL::CAN::Frame frame;
        frame.id = can_id;
        frame.dlc = dlc;
        frame.is_extended_id = false;
        frame.is_remote_frame = false;
        frame.mailbox = mailbox;
        
        memcpy(frame.data, data, dlc);
        can_bus.get_can1().send(frame);
    }
    
    /**
     * @brief 注册CAN接收回调到指定CAN设备
     * 
     * @param can_device HAL CAN设备实例
     */
    void registerCallback(HAL::CAN::ICanDevice* can_device)
    {
        if (can_device) {
            can_device->register_rx_callback([this](const HAL::CAN::Frame& frame) {
                // 创建临时CAN_RxHeaderTypeDef结构体以兼容现有Parse函数
                CAN_RxHeaderTypeDef rx_header;
                rx_header.StdId = frame.id;
                rx_header.IDE = frame.is_extended_id ? CAN_ID_EXT : CAN_ID_STD;
                rx_header.RTR = frame.is_remote_frame ? CAN_RTR_REMOTE : CAN_RTR_DATA;
                rx_header.DLC = frame.dlc;
                
                // 调用现有的Parse函数处理数据
                this->Parse(rx_header, frame.data);
            });
        }
    }
    /**
     * @brief 获取角度
     *
     * @param id can的id号，电机id - 初始id，例如3508的id为0x201，初始id为0x200，则id为0x201 - 0x200，也就是1,
     * @return float
     */
    float getAngleDeg(uint8_t id)
    {
        return this->unit_data_[id - 1].angle_Deg;
    }

    /**
     * @brief 获取弧度
     *
     * @param id can的id号，电机id - 初始id，例如3508的id为0x201，初始id为0x200，则id为0x201 - 0x200，也就是1,
     * @return float
     */
    float getAngleRad(uint8_t id)
    {
        return this->unit_data_[id - 1].angle_Rad;
    }

    /**
     * @brief 获取上一次角度
     *
     * @param id CAN id
     * @return float
     */
    float getLastAngleDeg(uint8_t id)
    {
        return this->unit_data_[id - 1].last_angle;
    }

    /**
     * @brief 获取增量角度
     *
     * @param id CAN id
     * @return float
     */
    float getAddAngleDeg(uint8_t id)
    {
        return this->unit_data_[id - 1].add_angle;
    }

    /**
     * @brief 获取增量弧度
     *
     * @param id CAN id
     * @return float
     */
    float getAddAngleRad(uint8_t id)
    {
        return this->unit_data_[id - 1].add_angle;
    }

    /**
     * @brief 获取速度    单位：(rad/s)
     * 这里是输出轴的速度，而不是转子速度
     * @param id CAN id
     * @return float
     */
    float getVelocityRads(uint8_t id)
    {
        return this->unit_data_[id - 1].velocity_Rad;
    }

    /**
     * @brief 获取速度    单位：(rpm)
     * 这里转子速度，不是输出轴的
     * @param id CAN id
     * @return float
     */
    float getVelocityRpm(uint8_t id)
    {
        return this->unit_data_[id - 1].velocity_Rpm;
    }

    float getDmRpm(uint8_t id)
    {
        return this->unit_data_[id - 1].velocity_Rad * 60 / (2 * 3.14159);
    }

    /**
     * @brief 获取电流值    单位：(A)
     *
     * @param id CAN id
     * @return float
     */
    float getCurrent(uint8_t id)
    {
        return this->unit_data_[id - 1].current_A;
    }

    /**
     * @brief 获取力矩    单位：(Nm)
     *
     * @param id CAN id
     * @return float
     */
    float getTorque(uint8_t id)
    {
        return this->unit_data_[id - 1].torque_Nm;
    }

    /**
     * @brief 获取温度    单位：(°)
     *
     * @param id CAN id
     * @return float
     */
    float getTemperature(uint8_t id)
    {
        return this->unit_data_[id - 1].temperature_C;
    }

    /**
     * @brief 获取在线状态
     *
     * @param id CAN od
     * @return true 断联
     * @return false 在线
     */
    bool GetDir(uint8_t id)
    {
        return this->runTime_[id - 1].Dir_Flag;
    }

    uint32_t getRunTime(uint8_t id)
    {
        return this->runTime_[id - 1].dirTime.lastTime;
    }
};
} // namespace BSP::Motor