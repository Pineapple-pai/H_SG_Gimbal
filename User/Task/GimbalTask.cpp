#include "../Task/GimbalTask.hpp"

#include "../APP/Data/GimbalData.hpp"
#include "../APP/Variable.hpp"
#include "../BSP/IMU/HI12H3_IMU.hpp"
#include "../BSP/Motor/DM/DmMotor.hpp"
#include "../BSP/Motor/Dji/DjiMotor.hpp"
#include "../BSP/Remote/Mini/Mini.hpp"
#include "../Task/CommunicationTask.hpp"

#include "../APP/Mod/RemoteModeManager.hpp"
#include "../BSP/DWT/DWT.hpp"

#include "can.h"
#include "cmsis_os2.h"
// ============ 云台控制参数 ============
// Pitch轴MIT控制参数
float pitch_Kp = 40.0f;
float pitch_Kd = 1.5f;

float pitch_vel_scale = 2.5f;
float pitch_vel = 0.0f;

float pitch_friction_comp = 0.20f; // 阻力补偿前馈
float pitch_vel_ff_kd = 0.0f;     // 角速度前馈增益

// Pitch轴状态变量
float filter_tar_pitch = 0.0f;
float target_tar_pitch = -120.0f;

// Yaw轴速度控制变量
float yaw_vel_scale = 3.0f;
float fliter_tar_yaw = 0.0f;

float yaw_friction_comp = 0.0f;
float yaw_vel_ff_kd = 0.0f;

// Yaw轴MIT控制参数
float yaw_kp = 40.0f;
float yaw_kd = 1.5f;
float yaw_vel_kd = 1.5f;
// 初始化相关
static bool gimbal_initialized = false;
static constexpr float YAW_INIT_ANGLE = 88.0f;

// 重力补偿系数（需要根据实际调试）
float gravity_comp = 1.35f;
float gravity_feedforward = 0.0f;

float zero_yaw = 0.0f;
void GimbalTask(void *argument)
{
    osDelay(500);
    
    static bool motor_enabled = false;
    for (;;)
    {
         if (!motor_enabled)
         {
            BSP::Motor::DM::Motor4310.On(1, BSP::Motor::DM::MIT);
            osDelay(5);  // 增加延时，确保指令发送成功
            BSP::Motor::DM::Motor4310.On(2, BSP::Motor::DM::MIT);
            osDelay(5);
            motor_enabled = true;
         }
        TASK::GIMBAL::gimbal.upDate();
        osDelay(4);
    }
}

namespace TASK::GIMBAL
{
// 构造函数定义，使用初始化列表
Gimbal::Gimbal()
    //: //adrc_yaw_vel(Alg::LADRC::TDquadratic(100, 0.005), 8, 40, 0.1, 0.005, 16384),
      // 速度pid的积分
      //pid_yaw_angle{0, 0}
      // pid的k值
      //kpid_yaw_angle{8, 0, 0}
{
    // 其他初始化逻辑（如果有）
}

void Gimbal::upDate()
{
    UpState();
    yawControl();

    pitchControl();

    sendCan();
}

void Gimbal::UpState()
{
    Status[Now_Status_Serial].Count_Time++; // 计时

    using namespace APP::Data;

    auto *remote = Mode::RemoteModeManager::Instance().getActiveController();

    auto remote_rx = remote->getRightX();
    auto remote_ry = remote->getRightY();

    // 获取当前角度值
    auto cur_angle = BSP::IMU::imu.getAddYaw();

    // 静态变量：记录上一次的模式，用于检测模式切换
    static int last_mode = GIMBAL::DISABLE;
    
    switch (Now_Status_Serial)
    {
    case (GIMBAL::DISABLE): {

        // 如果失能则让期望值等于实际值
        filter_tar_yaw_pos = BSP::IMU::imu.getAddYaw();
        filter_tar_yaw_vel = 0;  // 速度期望归零
        pitch_vel = 0;           // pitch速度期望归零

        filter_tar_pitch = BSP::Motor::DM::Motor4310.getAddAngleDeg(1);
        fliter_tar_yaw = BSP::Motor::DM::Motor4310.getAddAngleDeg(2);

        if (!gimbal_initialized)
        {
            fliter_tar_yaw = YAW_INIT_ANGLE;
            filter_tar_yaw_pos = YAW_INIT_ANGLE;
            gimbal_initialized = true;
        }
        else
        {
            fliter_tar_yaw = BSP::Motor::DM::Motor4310.getAddAngleDeg(2);
        }

        break;
    }
    case (GIMBAL::VISION): {
        // 视觉模式
        
        // 直接使用视觉位置控制，移除是否有目标的判断
        filter_tar_yaw_pos = Communicat::vision.getTarYaw();
        filter_tar_pitch = Communicat::vision.getTarPitch();
        
        break;
    }
    case (GIMBAL::KEYBOARD): {
        // 键鼠模式
        //filter_tar_yaw_vel = remote->getMouseVelX() * 100000;
        //filter_tar_pitch += remote->getMouseVelY() * 1000;
        pitch_vel = -remote_ry * pitch_vel_scale;
	    filter_tar_yaw_vel = remote_rx * yaw_vel_scale;
        // 一键掉头
        TurnAround();
        break;
    }
    case (GIMBAL::NORMAL): {
        filter_tar_yaw_vel = remote_rx * yaw_vel_scale;

        filter_tar_pitch += remote_ry * 0.5f;
        pitch_vel = -remote_ry * pitch_vel_scale;
        filter_tar_yaw_pos += filter_tar_yaw_vel * 0.004f;
        break;
    }
    }
    
    // 在函数末尾更新 last_mode，确保能正确检测模式切换
    last_mode = Now_Status_Serial;

    // if (is_sin == 0)
    // {
    //     gimbal_data.setTarYaw(tar_yaw.x1);
    // }
    // else if (is_sin == 1)
    // {
    //     sin_val = sinf(2 * 3.1415926f * HAL_GetTick() / 500.0f * sin_hz) * b;

    //     filter_tar_yaw_pos = sin_val;
    // }

    // pitch轴限幅
    filter_tar_pitch = Tools.clamp(filter_tar_pitch, -92.0f, -146.0f);
    target_tar_pitch = Tools.clamp(target_tar_pitch, -92.0f, -146.0f);

    // 期望值滤波
    tar_yaw.Calc(filter_tar_yaw_pos);
    tar_pitch.Calc(filter_tar_pitch);

    tar_yaw_vel.Calc(filter_tar_yaw_vel);
    tar_pitch_vel.Calc(pitch_vel);


    // 设置云台期望值
    gimbal_data.setTarYaw(tar_yaw.x1);
    gimbal_data.setTarPitch(tar_pitch.x1);
}

void Gimbal::yawControl()
{
    using namespace APP::Data;
    auto *remote = Mode::RemoteModeManager::Instance().getActiveController();

    // 陀螺仪反馈
    auto cur_yaw_angle = BSP::IMU::imu.getAddYaw();
    auto cur_yaw_vel = BSP::IMU::imu.getGyroZ() * 0.0174532f;

    // 根据模式选择控制策略
    if (Now_Status_Serial == GIMBAL::VISION)
    {
        BSP::Motor::DM::Motor4310.ctrl_Mit(2, (filter_tar_yaw_pos) * 0.0174532f, 0, yaw_kp, yaw_kd, 0);
    }
    else if(Now_Status_Serial == GIMBAL::NORMAL || Now_Status_Serial == GIMBAL::KEYBOARD)
    {
        // 小陀螺/键鼠模式：陀螺仪反馈 + ADRC速度环
        Adrc_yaw_vel.setTarget(tar_yaw_vel.x1);
        Adrc_yaw_vel.UpData(cur_yaw_vel);

        // 阻力补偿前馈 (带线性过渡区)
        float friction_torque = 0.0f;
        float deadband = 1.0f;

        if (fabs(tar_yaw_vel.x1) < deadband)
        {
            friction_torque = (tar_yaw_vel.x1 / deadband) * yaw_friction_comp;
        }
        else
        {
            friction_torque = (tar_yaw_vel.x1 > 0 ? 1 : -1) * yaw_friction_comp;
        }
        float final_torque = -Adrc_yaw_vel.getU() + friction_torque;

        BSP::Motor::DM::Motor4310.ctrl_Mit(2, 0, -tar_yaw_vel.x1 * 0.0174532f, 0, yaw_vel_ff_kd, final_torque);
        
        Tools.vofaSend(tar_yaw_vel.x1, cur_yaw_vel, BSP::Motor::DM::Motor4310.getAngleDeg(2), 
                final_torque, friction_torque, 0);
    }
    else if(Now_Status_Serial == GIMBAL::DISABLE)
    {
        // 失能模式
        BSP::Motor::DM::Motor4310.ctrl_Mit(2, 0, 0, 0, 0, 0);
    }
    // Tools.vofaSend(Communicat::vision.rx_target.yaw_angle, filter_tar_yaw_pos, 
    //                 BSP::Motor::DM::Motor4310.getAngleDeg(2), 
    //                 filter_tar_yaw_pos - BSP::Motor::DM::Motor4310.getAngleDeg(2), Communicat::vision.getTarYaw(), 0);

}


void Gimbal::pitchControl()
{
    using namespace  APP::Data;

    // 陀螺仪反馈
    auto cur_pitch_angle = BSP::IMU::imu.getPitch();                 // 角度反馈 (deg)
    auto cur_pitch_vel = BSP::IMU::imu.getGyroX() * 0.0174532f;      // 角速度反馈 (rad/s)
    gravity_feedforward = gravity_comp * cosf(cur_pitch_angle * 0.0174532f);
    if (Now_Status_Serial == GIMBAL::DISABLE)
    {
        BSP::Motor::DM::Motor4310.ctrl_Mit(1, 0, 0, 0, 0, 0);
    }
    else if(Now_Status_Serial == GIMBAL::VISION)
    {
        // 视觉模式：使用MIT位置控制
        BSP::Motor::DM::Motor4310.ctrl_Mit(1, (filter_tar_pitch) * 0.0174532f, 0, pitch_Kp, pitch_Kd, gravity_feedforward);
    }
    else if(Now_Status_Serial == GIMBAL::NORMAL || Now_Status_Serial == GIMBAL::KEYBOARD)
    { 
        // ADRC速度环控制
        Adrc_pitch_vel.setTarget(tar_pitch_vel.x1);
        Adrc_pitch_vel.UpData(cur_pitch_vel);

        

        float final_torque = Adrc_pitch_vel.getU() + gravity_feedforward;

        BSP::Motor::DM::Motor4310.ctrl_Mit(1, 0, 0, 0, 0, final_torque);
        
        // VOFA+ 调试信号: 目标速度, 反馈速度, ADRC输出, 阻力前馈, 重力前馈, 总力矩
        // Tools.vofaSend(tar_pitch_vel.x1, cur_pitch_vel, Adrc_pitch_vel.getU(), friction_torque, gravity_feedforward, final_torque);
    }

    Tools.vofaSend(Communicat::vision.getVisionPitch(), filter_tar_pitch,
                BSP::Motor::DM::Motor4310.getAngleDeg(1),
                filter_tar_pitch - BSP::Motor::DM::Motor4310.getAngleDeg(1), Communicat::vision.getTarPitch(), BSP::IMU::imu.getPitch());
}

void Gimbal::sendCan()
{
    // 预留CAN发送接口
}

void Gimbal::TurnAround()
{
    if (is_true_around == true)
    {
        // 360 deg/s为180度
        filter_tar_yaw_vel = 360.0f; // 可以根据需要调整速度大小

        // 如果旋转时间超过500ms，重置状态
        if (HAL_GetTick() - true_around_time > 500)
        {
            is_true_around = false;
            filter_tar_yaw_vel = 0;
        }
    }
}

} // namespace TASK::GIMBAL
