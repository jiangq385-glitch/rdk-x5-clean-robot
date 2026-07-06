#ifndef _EXCHANGE_H_
#define _EXCHANGE_H_

#include "main.h"
// 本模块负责交换/通信相关的数据组织。
// 机械臂下位机协议保持不变：关节目标位置以 rad 为原始单位，发送前按 abs(rad) * 1000 打包，
// 正负号由 flag 位表示。
typedef struct
{
    float acc[3];          // 三轴加速度 [ax, ay, az]（单位常见为 m/s^2）
		double vel[3];         // 三轴速度（由加速度积分得到）
	  float gyro[3];         // 三轴角速度 [gx, gy, gz]（单位常见为 °/s）
      
	  float PITCH_ANGLE;      // 当前俯仰角（Pitch，IM948 欧拉角 X）
	  float YAW_ANGLE;        // 当前偏航角（Yaw，IM948 欧拉角 Z）
	  float ROLL_ANGLE;       // 当前横滚角（Roll，IM948 欧拉角 Y）
	// 起始角（记录零点/校准时刻）
	  float PITCH_ANGLE_BEG;  // 俯仰起始角
	  float YAW_ANGLE_BEG;    // 偏航起始角
	  float ROLL_ANGLE_BEG;   // 横滚起始角
	// 相对变化量（当前角 - 起始角）
	  float PITCH_ANGLE_Del;  // 俯仰相对变化量
	  float YAW_ANGLE_Del;    // 偏航相对变化量
	  float ROLL_ANGLE_Del;   // 横滚相对变化量



	float quat[4];            // 四元数 [w, x, y, z]
	float ACCX_CALI;          // X 轴校准后加速度（便于调试）
	float ACCY_CALI;          // Y 轴校准后加速度
	float ACCZ_CALI;          // Z 轴校准后加速度
	  int cali ;              // 校准/使能标志位（项目自定义）
	
	 float REAL_YAW;          // 当前航向角（通常来自 IM948 的 YAW_ANGLE）
} IMU_DATA;

void exchange_task(void);
void Read_imu_data(void);
#endif // _EXCHANGE_H_
