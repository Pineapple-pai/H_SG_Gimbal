#include "../Task/ShootTask.hpp"
#include "../Task/CommunicationTask.hpp"

#include "../APP/Mod/RemoteModeManager.hpp"

#include "../APP/Tools.hpp"
#include "../APP/Variable.hpp"

#include "../BSP/Motor/Dji/DjiMotor.hpp"
#include "../BSP/Motor/DM/DmMotor.hpp"
#include "cmsis_os2.h"
float hz_send;
float sw_val = 0;
uint32_t firetime;
uint8_t firenum;
uint32_t fireMS;
float tar_angle = 0.0f;
uint32_t Send_ms = 0;
int16_t debug_limit = 100;
int16_t debug_cooling = 10;
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
    
    // 初始化单击状态机
    ClickFSM.Set_Status(Click_Status::CLICK_DISABLE);
    
    // 初始化停火状态机
    StopFireFSM.Set_Status(StopFire_Status::STOP_FIRE_DISABLE);
}

void Class_ClickFSM::UpState(bool key_pressed)
{
    Status[Now_Status_Serial].Count_Time++;

    const uint32_t time_threshold_counts = click_time_threshold / 5;

    switch (Now_Status_Serial)
    {
    case Click_Status::CLICK_DISABLE:
        if (key_pressed)
             Set_Status(Click_Status::PRESS_DOWN);
        break;

    case Click_Status::PRESS_DOWN:
        if (!key_pressed)
        {
            // 松开，判断时间
            if (Status[Now_Status_Serial].Count_Time < time_threshold_counts)
            {
                Set_Status(Click_Status::CLICK);
            }
            else
            {
                Set_Status(Click_Status::RELEASE); // 长按后松开 -> Release
            }
        }
        else
        {
            // 按住中，判断是否超时变为长按
            if (Status[Now_Status_Serial].Count_Time >= time_threshold_counts)
            {
                Set_Status(Click_Status::LONG_PRESS);
            }
        }
        break;

    case Click_Status::CLICK:
        // 单击状态是一次性的，下一帧直接跳回RELEASE或DISABLE
        Set_Status(Click_Status::RELEASE);
        break;

    case Click_Status::LONG_PRESS:
        if (!key_pressed)
        {
            Set_Status(Click_Status::RELEASE);
        }
        break;

    case Click_Status::RELEASE:
        if (key_pressed)
        {
            Set_Status(Click_Status::PRESS_DOWN);
        }
        // else 维持 RELEASE
        break;
        
    default:
        if (key_pressed) Set_Status(Click_Status::PRESS_DOWN);
        break;
    }
}

void Class_StopFireFSM::UpState(float current_torque, float time_elapsed_sec)
{
    Status[Now_Status_Serial].Count_Time++;
    
    // 1s threshold -> 200 counts (at 5ms)
    const uint32_t timeout_counts = 200; 

    switch (Now_Status_Serial)
    {
        case StopFire_Status::STOP_FIRE_DISABLE:
            // 等待外部激活
            break;
            
        case StopFire_Status::STOP_FIRE_ACTIVE:
            // 激活状态（发射中），监控电流和时间
            // 若摩擦轮当前扭矩电流超过阈值 OR 激活状态超过时间阈值 -> 进入处理状态
            if (fabs(current_torque) > stop_torque_threshold || Status[Now_Status_Serial].Count_Time > timeout_counts)
            {
                Set_Status(StopFire_Status::STOP_FIRE_PROCESSING);
            }
            break;
            
        case StopFire_Status::STOP_FIRE_PROCESSING:
             Set_Status(StopFire_Status::STOP_FIRE_DISABLE);
            break;
    }
}

void Class_JammingFSM::UpState()
{
    Status[Now_Status_Serial].Count_Time++; // 计时

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
        // 使用ADRC控制摩擦轮
        Adrc_Friction_L.setTarget(0.0f);
        Adrc_Friction_R.setTarget(0.0f);

		float current_angle = BSP::Motor::Dji::Motor2006.getAddAngleDeg(1);

        adrc_Dail_vel.setTarget(current_angle);
        break;
    }
    case (Booster_Status::ONLY): {
        // 使用ADRC控制摩擦轮速度
        Adrc_Friction_L.setTarget(-target_friction_omega);
        Adrc_Friction_R.setTarget(target_friction_omega);

        // 热量限制（滑动窗口，需要持续计算）
        HeatLimit();

        // 获取当前角度
        float current_angle = BSP::Motor::Dji::Motor2006.getAddAngleDeg(1);
        
        // 获取遥控器
        auto *remote = Mode::RemoteModeManager::Instance().getActiveController();

        // 获取SW开关值
        sw_val = remote->getSw();
        static float last_sw_val = 0; // 定义在外面，确保跨分支记录

        if (sw_val < 0)
        {
            // === 单发模式逻辑 (sw < 0.5) ===
            // 检测下降沿：上次>=0 且 本次<0
            bool sw_triggered = (last_sw_val >= 0);

            bool allow_fire = true; 
            allow_fire = 10;
            //Heat_Limit.getCurrentFireRate() > 0.0f;

            bool is_active_input = false;
            
            if (remote->isKeyboardMode())
            {
                // 键鼠模式：触发源 = 鼠标左键
                is_active_input = remote->getMouseKeyLeft(); 
                // 仅键鼠模式使用 ClickFSM
                ClickFSM.UpState(is_active_input);
                if (ClickFSM.isClick() && allow_fire)
                {
                    StopFireFSM.Activate();
                    Dail_target_pos -= 40.0f;
                }
            }
            else
            {
                 // 遥控器模式：直接使用 SW 切换沿触发
                 if (sw_triggered && allow_fire)
                 {
                     StopFireFSM.Activate();
                     Dail_target_pos -= 40.0f;
                 }
            }

            // 更新停火状态机
            float current_torque = BSP::Motor::Dji::Motor2006.getTorque(1);
            StopFireFSM.UpState(current_torque, 0);

            if (StopFireFSM.isProcessing())
            {
                Dail_target_pos = current_angle;
            }
        }
        else
        {
            // === 连发模式逻辑 (sw >= 0.5) ===
            target_fire_hz = sw_val * 25.0f; 

            if (remote->isKeyboardMode())
            {
                target_fire_hz = remote->getMouseKeyLeft() * 20.0f;
            }

            // 2. 应用热量限制
            // 修复：参数顺序 (val, min, max) -> (val, 0, max)
            target_fire_hz = Tools.clamp(target_fire_hz, 10.0f, 0.0f);

            // 3. 计算角度步长
            float angle_per_frame = hz_to_angle(target_fire_hz);

            // 4. 更新目标位置
            Dail_target_pos -= angle_per_frame;
            
            // 重要：在切回连发时，重置
            StopFireFSM.Reset();
        }

        last_sw_val = sw_val;
        break;
    }
    case (Booster_Status::AUTO): {
        // 连发模式，使用ADRC控制摩擦轮
        Adrc_Friction_L.setTarget(-target_friction_omega);
        Adrc_Friction_R.setTarget(target_friction_omega);

        auto *remote = Mode::RemoteModeManager::Instance().getActiveController();

        // 获取目标发射频率（Hz）
        target_fire_hz = remote->getSw() * 25.0f; // 最大20Hz

        if (remote->isKeyboardMode())
        {
            target_fire_hz = remote->getMouseKeyLeft() * 20.0f;
        }

        // 热量限制
        HeatLimit();

        // 应用热量限制
        target_fire_hz = Tools.clamp(target_fire_hz, 10.0f, 0.0f);

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
    // 使用ADRC控制摩擦轮
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
    //Tools.vofaSend(Heat_Limit.getCurrentHeat(), Heat_Limit.getHeatLimit(), Heat_Limit.getCurrentFireRate(), target_fire_hz, 0, 0);
    CAN_Send();
}

void Class_ShootFSM::HeatLimit()
{
    auto CurL = BSP::Motor::Dji::Motor3508.getCurrent(1);
    auto CurR = BSP::Motor::Dji::Motor3508.getCurrent(2);

    auto velL = BSP::Motor::Dji::Motor3508.getVelocityRpm(1);
    auto velR = BSP::Motor::Dji::Motor3508.getVelocityRpm(2);

	//如果发0则为断连
	// if(Gimbal_to_Chassis_Data.getBoosterHeatLimit() != 0)
	// {
	// 	Heat_Limit.setBoosterHeatParams(Gimbal_to_Chassis_Data.getBoosterHeatLimit(), Gimbal_to_Chassis_Data.getBoosterHeatCd());
	// }
    
    // 使用调试参数进行测试
    Heat_Limit.setBoosterHeatParams(debug_limit, debug_cooling);

    Heat_Limit.setFrictionCurrent(CurL, CurR);
    Heat_Limit.setFrictionVelocity(velL, velR);
    Heat_Limit.setTargetFireRate(target_fire_hz);

    Heat_Limit.UpDate();

}

void Class_ShootFSM::CAN_Send(void)
{
    // 使用ADRC输出控制摩擦轮
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
    const float control_period = 0.005f;                      // 控制周期5ms (与osDelay(5)匹配)

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
    static bool is_jammed = false;
    static uint32_t timer = 0;
    static float last_angle = 0;
    static uint8_t jam_count = 0; // 新增：堵转计数器，用于防抖

    // 1. 退弹模式的处理 (持续200ms反转)
    if (is_jammed)
    {
        target_Dail_torque = -5000;
        Dail_target_pos = angle; // 避免位置环积分过大
        
        if (HAL_GetTick() - timer > 200)
        {
            is_jammed = false;
            timer = HAL_GetTick();
            last_angle = angle;
            jam_count = 0; // 重置计数
        }
        return;
    }

    // 2. 检测周期控制 (每200ms一次)
    if (HAL_GetTick() - timer < 200) return;

    // 3. 堵转判定: 力矩饱和 && 误差大 && 角度几乎不动
    if (fabs(target_Dail_torque) > 8000 && fabs(err) > 30 && fabs(angle - last_angle) < 10)
    {
        jam_count++;
        // 只有连续2次（400ms）检测到堵转才触发，防止单发启动瞬间误触
        if (jam_count >= 2)
        {
            is_jammed = true;
            jam_count = 0;
        }
        timer = HAL_GetTick(); // 记录时间
    }
    else
    {
        // 未检测到堵转，更新基准，重置计数
        last_angle = angle;
        timer = HAL_GetTick();
        jam_count = 0;
    }
}

bool Class_ShootFSM::getFrictionState()
{
    // 如果处于STOP或DISABLE状态，或者目标速度为0，则认为关闭
    if (Now_Status_Serial == Booster_Status::DISABLE || 
        Now_Status_Serial == Booster_Status::STOP)
    {
        return false;
    }
    return true;
}

int16_t Class_ShootFSM::getProjectileCount()
{
    return (int16_t)Heat_Limit.getFireCount();
}

} // namespace TASK::Shoot
