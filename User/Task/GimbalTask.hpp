#pragma once

#include "../Algorithm/FSM/alg_fsm.hpp"
#include "../User/BSP/SimpleKey/SimpleKey.hpp"

namespace TASK::GIMBAL
{

enum Gimbal_Status
{
    DISABLE,
    NORMAL,
    VISION,
    KEYBOARD
};

class Gimbal : public Class_FSM
{

  public:
    // 构造函数声明
    Gimbal();
    void upDate();

  private:
    void UpState();

    void filter();

    void pitchControl();
    void yawControl();

    void sendCan();

    float filter_tar_yaw_vel;
    float filter_tar_yaw_pos;



    // 按键状态
    BSP::Key::SimpleKey DM_state;
    BSP::Key::SimpleKey vision_state;

    bool is_true_around = false;
    uint32_t true_around_time = 0;
    
    // 视觉模式下是否有有效目标
    bool vision_has_target = false;

  public:
    void TurnAround();

    void setNowStatus(Gimbal_Status state)
    {
        Set_Status(state);
    }

    void setTrueAround()
    {
        is_true_around = true;
        true_around_time = HAL_GetTick();
    }
};

inline Gimbal gimbal;

} // namespace TASK::GIMBAL
// 将RTOS任务引至.c文件
#ifdef __cplusplus
extern "C"
{
#endif
    void GimbalTask(void *argument);

#ifdef __cplusplus
}
#endif