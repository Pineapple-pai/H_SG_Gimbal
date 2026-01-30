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
    // 失能
    DISABLE,
    // 停止
    STOP,
    // 单发模式
    ONLY,
    // 连发模式
    AUTO
};

/**
 * @brief 防卡弹状态类型
 *
 */
enum Jamming_Status
{
    NORMAL = 0, // 正常模式
    SUSPECT,    // 可疑堵转
    PROCESSING, // 处理堵转
};

class Class_ShootFSM;

class Class_JammingFSM : public Class_FSM
{
  private:
    // 堵转力矩
    float stall_torque = 0.2f;
    // 堵转时间阈值，超过则为堵转
    static constexpr uint32_t stall_time = 200;
    // 从堵转停止时间阈值，超过则停止堵转
    static constexpr uint32_t stall_stop = 300;

    Class_ShootFSM *Booster = nullptr; // 任务指针

  public:
    void UpState(void);

    // 添加设置Booster指针的方法
    void setBooster(Class_ShootFSM *booster)
    {
        Booster = booster;
    }
};

/**
 * @brief 单击检测状态类型
 */
enum Click_Status
{
    CLICK_DISABLE, // 失能
    PRESS_DOWN,    // 按下
    CLICK,         // 单击
    LONG_PRESS,    // 长按
    RELEASE,       // 放开
};

/**
 * @brief 单击有限状态机
 */
class Class_ClickFSM : public Class_FSM
{
  public:
    void UpState(bool key_pressed);
    
    // 获取当前是否为单击状态
    bool isClick() { return Now_Status_Serial == Click_Status::CLICK; }

  private:
    static constexpr uint32_t click_time_threshold = 200; // 单击时间阈值 (ms)
    uint32_t press_start_time = 0;
};

/**
 * @brief 停火检测状态类型
 */
enum StopFire_Status
{
    STOP_FIRE_DISABLE,   // 失能（默认）
    STOP_FIRE_ACTIVE,    // 激活（发射中）
    STOP_FIRE_PROCESSING,// 处理（刹车/停火）
};

/**
 * @brief 停火有限状态机
 */
class Class_StopFireFSM : public Class_FSM
{
  public:
    void UpState(float current_torque, float time_elapsed_sec);

    // 重置状态机到失能状态
    void Reset() { Set_Status(StopFire_Status::STOP_FIRE_DISABLE); }
    
    // 激活状态机
    void Activate() { Set_Status(StopFire_Status::STOP_FIRE_ACTIVE); }

    // 是否需要处理（刹车）
    bool isProcessing() { return Now_Status_Serial == StopFire_Status::STOP_FIRE_PROCESSING; }

  private:
    // 停火电流/力矩阈值 (对应截图中的8A，这里单位即使是力矩值也需要转换或根据实际单位设定)
    // 假设 AD值 8A 对应 DjiMotor 的 current/torque 单位，需根据实际情况调整
    // 一般 3508/2006 的电流反馈 16384 对应 20A， 8A 约为 6553
    // 但此处传入的是 force/torque，需确认。假设传入的是 torque (current)
    const float stop_torque_threshold = 8000.0f; // 暂定阈值
    const float stop_time_threshold = 1.0f;      // 1s 超时
};

class Class_ShootFSM : public Class_FSM
{
  public:
    // 显式声明构造函数
    Class_ShootFSM();

    // ADRC控制器
    void Control(void);

    void UpState(void);

    void setTargetDailTorque(float torque)
    {
        target_Dail_torque = torque;
    }

    void setNowStatus(Booster_Status state)
    {
        Set_Status(state);
    }

    // 设置开火标志位
    void setFireFlag(bool flag)
    {
        fire_flag = flag;
    }

    bool getFrictionState();
    int16_t getProjectileCount();
    
  protected:
    // 初始化相关常量

    // 热量控制

    // 检测摩擦轮力矩变化

    // 拨盘控制

    // CAN发送
    void CAN_Send(void);
    void HeatLimit();

    // 将期望发射频率转化为rpm(转轴)
    float rpm_to_hz(float tar_hz);

    //  将期望频率转化为角度
    float hz_to_angle(float fire_hz);
    void Jamming(float angle, float err);

  private:
    float target_Dail_torque = 0;
    float Dail_target_pos;
    float target_friction_L_torque = 0;
    float target_friction_R_torque = 0;

    float target_friction_omega = 5900.0f;
    float target_torque = 1.5f;
    float target_fire_hz;
    float Max_dail_angle = 25.0f; // 拨盘最快频率
    float Motor_Friction_L_Out = 0.0f;
    float Motor_Friction_R_Out = 0.0f;
    
    Class_JammingFSM JammingFMS;
    Class_ClickFSM ClickFSM;        // 新增单击检测状态机
    Class_StopFireFSM StopFireFSM;  // 新增停火检测状态机

    // 开火标志位
    uint8_t fire_flag = 0;

    // APP::Heat_Detector::Class_FSM_Heat_Limit Heat_Limit;
    HeatControl::HeatController Heat_Limit;
    // 发射机构控制模式
    // Adrc Adrc_Friction_L;
    // Adrc Adrc_Friction_R;
    Adrc adrc_Dail_vel;

    // Kpid_t Kpid_Dail_pos;
    // Kpid_t Kpid_Dail_vel;

    // PID pid_Dail_pos;
    // PID pid_Dail_vel;

    // Kpid_t Kpid_Friction_L_vel;
    PID pid_Motor_Friction_L_vel;

    // Kpid_t Kpid_Friction_R_vel;
    PID pid_Motor_Friction_R_vel;



    // 用于单发检测，获取上升沿判断是否击发子弹
    // BSP::Key::SimpleKey key_fire;
};
inline Class_ShootFSM shoot_fsm;
} // namespace TASK::Shoot

// 将RTOS任务引至.c文件
#ifdef __cplusplus
extern "C"
{
#endif

    void ShootTask(void *argument);

#ifdef __cplusplus
}
#endif