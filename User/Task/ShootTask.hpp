#pragma once

#include "../Algorithm/FSM/alg_fsm.hpp"
#include "../Algorithm/LADRC/Adrc.hpp"
#include "../BSP/SimpleKey/SimpleKey.hpp"
#include "../User/Algorithm/PID.hpp"
#include "../User/Algorithm/LADRC/Adrc.hpp"
#include "../User/BSP/DWT/DWT.hpp"
#include "../APP/Variable.hpp"
#include "../APP/Heat_Detector/Heat_Control.hpp"

namespace TASK::Shoot
{
using namespace Alg::LADRC;

enum Booster_Status
{
    // 发射机构失能
    DISABLE,
    // 停止状态
    STOP,
    // 单发模式
    ONLY,
    // 连发模式
    AUTO
};

/**
 * @brief 卡弹检测状态
 */
enum Jamming_Status
{
    NORMAL = 0, // 正常
    SUSPECT,    // 疑似卡弹
    PROCESSING, // 卡弹处理中
};

class Class_ShootFSM;

class Class_JammingFSM : public Class_FSM
{
  private:
    // 判定卡弹的拨盘力矩阈值
    float stall_torque = 0.2f;
    // 力矩超阈持续时间，超过则进入处理
    static constexpr uint32_t stall_time = 200;
    // 卡弹处理持续时间
    static constexpr uint32_t stall_stop = 300;

    Class_ShootFSM *Booster = nullptr; // 发射状态机指针

  public:
    void UpState(void);

    // 绑定发射状态机实例
    void setBooster(Class_ShootFSM *booster)
    {
        Booster = booster;
    }
};

/**
 * @brief 单击检测状态
 */
enum Click_Status
{
    CLICK_DISABLE, // 未按下
    PRESS_DOWN,    // 按下中
    CLICK,         // 单击触发
    LONG_PRESS,    // 长按触发
    RELEASE,       // 释放
};

/**
 * @brief 单击/长按检测状态机
 */
class Class_ClickFSM : public Class_FSM
{
  public:
    void UpState(bool key_pressed);

    // 是否刚触发单击事件
    bool isClick() { return Now_Status_Serial == Click_Status::CLICK; }

  private:
    static constexpr uint32_t click_time_threshold = 200; // 单击时长阈值(ms)
    uint32_t press_start_time = 0;
};

/**
 * @brief 停火检测状态
 */
enum StopFire_Status
{
    STOP_FIRE_DISABLE,    // 失能（默认）
    STOP_FIRE_ACTIVE,     // 激活（发射中）
    STOP_FIRE_PROCESSING, // 处理中（刹车/停火）
};

/**
 * @brief 停火状态机
 */
class Class_StopFireFSM : public Class_FSM
{
  public:
    void UpState(float current_torque, float time_elapsed_sec);

    // 重置为失能
    void Reset() { Set_Status(StopFire_Status::STOP_FIRE_DISABLE); }

    // 进入激活
    void Activate() { Set_Status(StopFire_Status::STOP_FIRE_ACTIVE); }

    // 是否处于处理阶段
    bool isProcessing() { return Now_Status_Serial == StopFire_Status::STOP_FIRE_PROCESSING; }

  private:
    // 停火判据：力矩阈值（需结合电机反馈单位校准）
    const float stop_torque_threshold = 8000.0f;
    // 停火超时阈值（秒）
    const float stop_time_threshold = 1.0f;
};

class Class_ShootFSM : public Class_FSM
{
  public:
    Class_ShootFSM();

    // 发射机构控制主函数
    void Control(void);

    // 状态更新
    void UpState(void);

    void setTargetDailTorque(float torque)
    {
        target_Dail_torque = torque;
    }

    void setNowStatus(Booster_Status state)
    {
        Set_Status(state);
    }

    // 设置外部开火标志（视觉/上层逻辑）
    void setFireFlag(bool flag)
    {
        fire_flag = flag;
    }

    void setFrictionEnabled(bool enabled)
    {
        friction_enabled = enabled;
    }

    bool isFrictionEnabled() const
    {
        return friction_enabled;
    }

    bool getFrictionState();
    int16_t getProjectileCount();

  protected:
    void CAN_Send(void);
    void HeatLimit();

    // 将目标发射频率转换为拨盘转速（rpm）
    float rpm_to_hz(float tar_hz);

    // 将目标发射频率转换为每周期角度增量
    float hz_to_angle(float fire_hz);

    // 卡弹检测与处理
    void Jamming(float angle, float err);

  public:
    float target_Dail_torque = 0;
    float Dail_target_pos;
    float target_friction_L_torque = 0;
    float target_friction_R_torque = 0;

    float target_friction_omega = 5900.0f;
    float target_torque = 1.5f;
    float target_fire_hz;
    float Max_dail_angle = 25.0f; // 拨盘最大发射频率对应角增量
    float Motor_Friction_L_Out = 0.0f;
    float Motor_Friction_R_Out = 0.0f;

    Class_JammingFSM JammingFMS;
    Class_ClickFSM ClickFSM;
    Class_StopFireFSM StopFireFSM;

    // 开火标志位
    uint8_t fire_flag = 0;
    bool friction_enabled = false;

    HeatControl::HeatController Heat_Limit;

    // 发射机构控制器
    Adrc adrc_Dail_vel;
    PID pid_Motor_Friction_L_vel;
    PID pid_Motor_Friction_R_vel;

    // 单发/长按逻辑状态
    uint32_t trigger_start_tick = 0; // 扳机按下起始时刻
    bool last_trigger_state = false; // 上一周期扳机状态
    bool is_long_press_auto = false; // 是否进入长按连发
    uint32_t fire_confirm_count = 0; // 击发确认计数快照

    // 视觉开火：上升沿先保底打满一轮 burst，高电平保持时再转连续发射
    bool last_vision_fire_flag = false;
    bool vision_burst_active = false;
    uint8_t vision_burst_shot_count = 3;
    uint32_t vision_burst_start_fire_count = 0;
    static constexpr float dail_angle_per_shot = 40.0f;

    // BSP::Key::SimpleKey key_fire;
};

inline Class_ShootFSM shoot_fsm;
} // namespace TASK::Shoot

// 导出 RTOS 任务入口给 C 文件
#ifdef __cplusplus
extern "C"
{
#endif

    void ShootTask(void *argument);

#ifdef __cplusplus
}
#endif
