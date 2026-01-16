#include "../Task/ShootTask.hpp"
#include "../Task/CommunicationTask.hpp"

#include "../APP/Mod/RemoteModeManager.hpp"

#include "../APP/Tools.hpp"

#include "../BSP/Motor/Dji/DjiMotor.hpp"

#include "cmsis_os2.h"
float hz_send;

uint32_t firetime;
uint8_t firenum;
uint32_t fireMS;
float tar_angle = 0.0f;
uint32_t Send_ms = 0;
void ShootTask(void *argument)
{
    for (;;)
    {
        hz_send += 0.001;
        Communicat::vision.time_demo();

        // 设置视觉开火位
        TASK::Shoot::shoot_fsm.setFireFlag(Communicat::vision.get_fire_num());
//                firetime++;
//                if (firetime > fireMS)
//                {
//                    firenum = 1;
//                    firetime = 0;
//                }
//                else
//                {
//                    firenum = 0;
//                }
//                TASK::Shoot::shoot_fsm.setFireFlag(firenum);

        TASK::Shoot::shoot_fsm.Control();

        osDelay(5); 
    }
}

namespace TASK::Shoot
{
// 构造函数定义，使用初始化列表
Class_ShootFSM::Class_ShootFSM()
    : 
      adrc_Dail_vel(Alg::LADRC::TDquadratic(200, 0.001), 5, 40, 0.9, 0.001, 16384),
      // 位置pid增益
      //Kpid_Dail_pos(10, 0, 0),
      // 速度pid增益
      //Kpid_Dail_vel(200, 0, 0),
      // 热量限制初始化
      Heat_Limit(100, 55.0f) // 示例参数：窗口大小50，阈值10.0
{
    // 初始化卡弹检测状态机
    JammingFMS.Set_Status(Jamming_Status::NORMAL);
    // 将当前射击状态机实例传递给卡弹检测状态机
    JammingFMS.setBooster(this);
}

void Class_JammingFSM::UpState()
{
    Status[Now_Status_Serial].Count_Time++;

    auto Motor_Friction = BSP::Motor::Dji::Motor3508;
    auto Motor_Dail = BSP::Motor::Dji::Motor2006;

    switch (Now_Status_Serial)
    {
    case (Jamming_Status::NORMAL): {
        // 正常状态
        if (fabs(Motor_Dail.getTorque(1)) >= stall_torque)
        {
            // 大扭矩->卡弹嫌疑状态
            Set_Status(Jamming_Status::SUSPECT);
        }

        break;
    }
    case (Jamming_Status::SUSPECT): {
        // 卡弹嫌疑状态

        if (Status[Now_Status_Serial].Count_Time >= stall_time)
        {
            // 长时间大扭矩->卡弹反应状态
            Set_Status(Jamming_Status::PROCESSING);
        }
        else if (Motor_Dail.getTorque(1) < stall_torque)
        {
            // 短时间大扭矩->正常状态
            Set_Status(Jamming_Status::NORMAL);
        }

        break;
    }
    case (Jamming_Status::PROCESSING): {
        // 卡弹反应状态->准备卡弹处理
        Booster->setTargetDailTorque(5000);

        if (Status[Now_Status_Serial].Count_Time > stall_stop)
            Set_Status(Jamming_Status::NORMAL);

        break;
    }
    }
}

void Class_ShootFSM::UpState()
{
    switch (Now_Status_Serial)
    {
    case (Booster_Status::DISABLE): {
        // 如何失能状态，拨盘力矩为0，摩擦轮期望值为0
        Adrc_Friction_L.setTarget(0.0f);
        Adrc_Friction_R.setTarget(0.0f);

		float current_angle = BSP::Motor::Dji::Motor2006.getAddAngleDeg(1);

        adrc_Dail_vel.setTarget(current_angle);
        break;
    }
    case (Booster_Status::ONLY): {
        // 单发模式
        // 设置摩擦轮速度，与连发模式相同
        Adrc_Friction_L.setTarget(-target_friction_omega);
        Adrc_Friction_R.setTarget(target_friction_omega);

        // 热量限制（滑动窗口，需要持续计算）
        HeatLimit();

        // 获取当前角度
        float current_angle = BSP::Motor::Dji::Motor2006.getAddAngleDeg(1);

        static bool fired = false;
        static bool allow_fire = true; // 添加热量限制允许发射标志

        // 热量限制检查
        // getNowFire()返回限制后的发射频率（Hz），如果为0表示禁止发射
        allow_fire = Heat_Limit.getCurrentFireRate() > 0.0f;

        if (fire_flag == 1 && allow_fire)
        {
            if (!fired)
            {
                // 设置目标位置
                Dail_target_pos -= 40.0f;
                fired = true;
            }
            else
            {
                // 自动复位开火标志位
                fire_flag = 0;
            }
        }
        else
        {
            // 重置发射状态
            fired = false;
        }

        break;
    }
    case (Booster_Status::AUTO): {
        // 连发模式
        Adrc_Friction_L.setTarget(-target_friction_omega);
        Adrc_Friction_R.setTarget(target_friction_omega);

        auto *remote = Mode::RemoteModeManager::Instance().getActiveController();

        // 获取目标发射频率（Hz）
        target_fire_hz = remote->getSw() * 20.0f; // 最大20Hz

        if (remote->isKeyboardMode())
        {
            target_fire_hz = remote->getMouseKeyLeft() * 20.0f;
        }

        // 热量限制
        HeatLimit();

        // 应用热量限制
        target_fire_hz = Tools.clamp(target_fire_hz, Heat_Limit.getCurrentFireRate(), 0.0f);

        // 获取当前角度
        float current_angle = BSP::Motor::Dji::Motor2006.getAddAngleDeg(1);

        // 计算角度变化
        float angle_per_frame = hz_to_angle(target_fire_hz);

        // 更新目标位置
        Dail_target_pos -= angle_per_frame;

        break;
    }
    }
}

void Class_ShootFSM::Control(void)
{
    auto velL = BSP::Motor::Dji::Motor3508.getVelocityRpm(1);
    auto velR = BSP::Motor::Dji::Motor3508.getVelocityRpm(2);
    auto DailVel = BSP::Motor::Dji::Motor2006.getVelocityRpm(1);
    auto Dail_pos = BSP::Motor::Dji::Motor2006.getAddAngleDeg(1);
    UpState();
    // 控制摩擦轮
    Adrc_Friction_L.UpData(velL);
    Adrc_Friction_R.UpData(velR);

    // 如果卡弹就让期望等于反馈
    if (JammingFMS.Get_Now_Status_Serial() == Jamming_Status::PROCESSING)
        Dail_target_pos = Dail_pos;

    // 更新角度pid
    tar_angle += BSP::Remote::dr16.remoteLeft().x;
    pid_Dail_pos.setTarget(Dail_target_pos);
    pid_Dail_pos.GetPidPos(Kpid_Dail_pos, Dail_pos, 16384.0f);

    // 更新速度pid
    pid_Dail_vel.setTarget(pid_Dail_pos.getOut());
    pid_Dail_vel.GetPidPos(Kpid_Dail_vel, DailVel, 16384.0f);

    target_Dail_torque = pid_Dail_vel.getOut();

    // 卡弹检测
    Jamming(Dail_pos, pid_Dail_pos.GetErr());
    // 拨盘发送
//    Tools.vofaSend(Dail_target_pos, Dail_pos, fire_flag, Heat_Limit.getFireNum(), 0, 0);
    // 摩擦轮发送
    // Tools.vofaSend(Adrc_Friction_L.getZ1(), Adrc_Friction_L.getTarget(),
    // Adrc_Friction_L.getFeedback(),
    //                Adrc_Friction_R.getZ1(), Adrc_Friction_R.getTarget(),
    //                Adrc_Friction_R.getFeedback());

    // // 火控
    Tools.vofaSend(Adrc_Friction_R.getZ1(),BSP::Motor::Dji::Motor3508.getVelocityRpm(2),
                    target_friction_omega, 0, 0, 0);

    CAN_Send();
}

void Class_ShootFSM::HeatLimit()
{
    auto CurL = BSP::Motor::Dji::Motor3508.getTorque(1);
    auto CurR = BSP::Motor::Dji::Motor3508.getTorque(2);

    auto velL = BSP::Motor::Dji::Motor3508.getVelocityRpm(1);
    auto velR = BSP::Motor::Dji::Motor3508.getVelocityRpm(2);

	//如果发0则为断连
	if(Gimbal_to_Chassis_Data.getBoosterHeatLimit() != 0)
	{
		Heat_Limit.setBoosterHeatParams(Gimbal_to_Chassis_Data.getBoosterHeatLimit(), Gimbal_to_Chassis_Data.getBoosterHeatCd());
	}
//    Heat_Limit.setBoosterHeat(180, 40);

    Heat_Limit.setFrictionCurrent(CurL, CurR);
    Heat_Limit.setFrictionVelocity(velL, velR);
    // Heat_Limit.setTargetFire(target_fire_hz);

    //Heat_Limit.UpDate();
}

void Class_ShootFSM::CAN_Send(void)
{
    BSP::Motor::Dji::Motor3508.setCAN(Adrc_Friction_L.getU(), 2);
    BSP::Motor::Dji::Motor3508.setCAN(Adrc_Friction_R.getU(), 3);
    BSP::Motor::Dji::Motor3508.setCAN(target_Dail_torque, 1);
    
    // if(Send_ms == 0)
    // {
        BSP::Motor::Dji::Motor3508.sendCAN();
    // }
    Send_ms++;
    Send_ms %= 2; 

}

float Class_ShootFSM::rpm_to_hz(float tar_hz)
{
    const int slots_per_rotation = 9;       // 拨盘每转一圈的槽位数
    const double seconds_per_minute = 60.0; // 每分钟的秒数

    // 计算每秒需要的转数
    double rotations_per_second = tar_hz / slots_per_rotation;

    // 转换为每分钟转数（RPM）
    double rpm = rotations_per_second * seconds_per_minute;

    return rpm;
}

// 添加一个新函数，将Hz转换为角度增量，保持线性关系
float Class_ShootFSM::hz_to_angle(float fire_hz)
{
    const int slots_per_rotation = 9;                         // 拨盘每转一圈的槽位数
    const float angle_per_slot = 360.0f / slots_per_rotation; // 每个槽位对应的角度
    const float control_period = 0.001f;                      // 控制周期1ms

    // 计算每帧需要转动的角度
    // (目标频率 * 每发角度) / 控制周期
    float angle_per_frame = (fire_hz * angle_per_slot) * control_period;

    return angle_per_frame;
}

/**
 * @brief 通过位置控制的拨盘卡弹检测更加灵敏
 * 
 * @param angle 
 * @param err 
 */
void Class_ShootFSM::Jamming(float angle, float err)
{
    static uint8_t blocking_flag = 0;
    static uint32_t blocking_time = 0;
    static uint32_t check_time = 0;
    static float last_angle = 0;
    
    // 初始化
    if (check_time == 0)
    {
        check_time = HAL_GetTick();
        last_angle = angle;
        return;
    }

    // 退弹处理中
    if (blocking_flag)
    {
        if (HAL_GetTick() - blocking_time > 200)
        {
            blocking_flag = 0;
            last_angle = angle;
            check_time = HAL_GetTick();
        }
        else
        {
            Dail_target_pos = angle;
            target_Dail_torque = -5000;  // 反向退弹
        }
        return;
    }

    // 每 200ms 检测一次
    if (HAL_GetTick() - check_time < 200)
        return;

    // 堵转判定条件：
    // 1. 输出力矩大
    // 2. 位置误差大
    // 3. 角度变化小
    float angle_change = fabs(angle - last_angle);
    bool high_torque = fabs(target_Dail_torque) > 8000;
    bool large_error = fabs(err) > 30;
    bool not_moving = angle_change < 10;

    if (high_torque && large_error && not_moving)
    {
        blocking_flag = 1;
        blocking_time = HAL_GetTick();
    }

    // 更新检测基准
    last_angle = angle;
    check_time = HAL_GetTick();
}


} // namespace TASK::Shoot
