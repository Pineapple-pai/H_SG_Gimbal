#include "Lk_motor.hpp"
LK_Motor::LK_Motor(int16_t address, uint8_t MotorSize, LK_Motor_Data *MotorAddress, uint8_t *idxs)
{
    this->_Motor_ID_IDX_BIND_(idxs, MotorSize);
    
    this->motorData = MotorAddress;
    this->init_address = address;
    for (uint8_t i = 0; i < MotorSize; i++)
    {
        this->motorData[i].is_enabled = false;
        this->motorData[i].address = address + idxs[i];
        this->motorData[i].allow_accumulate = false;
        this->motorData[i].total_angle = 0.0f;
        this->motorData[i].InitFlag = 0;
    }

    this->MotorSize = MotorSize;
    this->Date_IDX = 0;
}
//数据解析
void LK_Motor::Parse(RM_FDorCAN_RxHeaderTypeDef RxHeader, uint8_t RxHeaderData[])
{
    if (!(FDorCAN_ID(RxHeader) >= this->init_address && FDorCAN_ID(RxHeader) <= this->init_address + 10) || this->MotorSize == 0)
        return;

    int idx = GET_Motor_ID_ADDRESS_BIND_(FDorCAN_ID(RxHeader));

    if (idx == -1)
        return; // 如果超越数组大小，或者不存在id

    this->motorData[idx].DirFlag = this->motorData[idx].dirTime.ISDir(10);

    uint8_t cmd = RxHeaderData[0];

    // 温度
    this->motorData[idx].Data[LK_Temperature] = (float)((int16_t)RxHeaderData[1]);
    
    // 转矩电流
    int16_t iq_raw = (int16_t)((RxHeaderData[3] << 8) | RxHeaderData[2]);
    this->motorData[idx].Data[LK_Torque] = (float)iq_raw * (66.0f / 4096.0f);

    // 速度
    this->motorData[idx].Data[LK_Speed] = (float)((int16_t)((RxHeaderData[5] << 8) | RxHeaderData[4]));

    // 获取当前角度
    float Tangle = (uint16_t)((RxHeaderData[7] << 8) | RxHeaderData[6]);
    this->motorData[idx].Data[LK_Angle] = (float)(int16_t)Tangle * (360.0f / 65535.0f);

    // 数据累加 - 多圈角度计算
    if (this->motorData[idx].allow_accumulate) {
        if (this->motorData[idx].LastData[LK_Angle] != this->motorData[idx].Data[LK_Angle] && this->motorData[idx].LastData[LK_Angle] != -1)
        {
            float lastData = this->motorData[idx].LastData[LK_Angle];
            float Data = this->motorData[idx].Data[LK_Angle];

            float delta = Data - lastData;
            
            // 处理360°跳变
            if (delta > 180.0f) {	
                delta -= 360.0f;
            } else if (delta < -180.0f) {
                delta += 360.0f;
            }
            
            this->motorData[idx].total_angle += delta;
            this->motorData[idx].AddData += delta;
        }
    }

    // 保存上一次数据
    for (int i = 0; i < 4; i++) {
        this->motorData[idx].LastData[i] = this->motorData[idx].Data[i];
    }

    // 初始化数据
    if (this->motorData[idx].InitFlag == 0)
    {
        this->motorData[idx].InitData = this->motorData[idx].Data[LK_Angle];
        this->motorData[idx].InitFlag = 1;
    }
    
    // 更新时间
    this->motorData[idx].dirTime.UpLastTime();
}
// 使能电机
//使能电机
void LK_Motor::ON(RM_FDorCAN_HandleTypeDef *hcan)
{
    this->send_data[0] = 0x88;
    for(int i = 1; i < 8; i++) {
        this->send_data[i] = 0x00;
    }
    RM_FDorCAN_Send(hcan, this->motorData[0].address, this->send_data, CAN_TX_MAILBOX1);
}

//失能电机
void LK_Motor::OFF(RM_FDorCAN_HandleTypeDef *hcan)
{
    this->send_data[0] = 0x81;
    for(int i = 1; i < 8; i++) {
        this->send_data[i] = 0x00;
    }
    RM_FDorCAN_Send(hcan, this->motorData[0].address, this->send_data, CAN_TX_MAILBOX1);
}

//清除错误
void LK_Motor::clear_err(RM_FDorCAN_HandleTypeDef *hcan)
{
    this->send_data[0] = 0x9B;
    for(int i = 1; i < 8; i++) {
        this->send_data[i] = 0x00;
    }
    RM_FDorCAN_Send(hcan, this->motorData[0].address, this->send_data, CAN_TX_MAILBOX1);
}
void LK_Motor::SetPositionCrtl(RM_FDorCAN_HandleTypeDef *hcan, int32_t angle, uint16_t speed)
{

    uint8_t data[8];
    uint32_t encoder_value = (angle * 100);        
    this->send_data[0] = 0xA4; 							// 命令字节
    this->send_data[1] = 0x00;                          // 保留字节
    this->send_data[2] = speed & 0xFF;                  // 速度限制低字节
    this->send_data[3] = (speed >> 8) & 0xFF;           // 速度限制高字节
    this->send_data[4] = encoder_value & 0xFF;          // 位置控制低字节
    this->send_data[5] = (encoder_value >> 8) & 0xFF;   // 位置控制高字节
    this->send_data[6] = (encoder_value >> 16) & 0xFF;  // 位置控制高字节
    this->send_data[7] = (encoder_value >> 24) & 0xFF;  // 位置控制高字节

    RM_FDorCAN_Send(hcan, this->motorData[0].address, this->send_data, CAN_TX_MAILBOX1);
}

void LK_Motor::SetTorqueCtrl(RM_FDorCAN_HandleTypeDef *hcan, int16_t torque)
{
    this->send_data[0] = 0xA1;                   // 命令字节
    this->send_data[1] = 0x00;
    this->send_data[2] = 0x00;    
    this->send_data[4] = torque & 0xFF;          // 力矩控制低字节
    this->send_data[5] = (torque >> 8) & 0xFF;   // 力矩控制高字节
    this->send_data[6] = 0x00;                    
    this->send_data[7] = 0x00;                       

    RM_FDorCAN_Send(hcan, this->motorData[0].address, this->send_data, CAN_TX_MAILBOX1);
}

uint8_t LK_Motor::ISDir()
{
	bool is_dir = 0;
	for (int i = 0; i < this->MotorSize; i++)
	{
		is_dir |= this->motorData[GET_Motor_ID_ADDRESS_BIND_(this->motorData[i].address)].DirFlag =
				this->motorData[GET_Motor_ID_ADDRESS_BIND_(this->motorData[i].address)].dirTime.ISDir(100);
	}
	return is_dir;
}

