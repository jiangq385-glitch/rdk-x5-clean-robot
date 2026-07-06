#ifndef __ODOM_H
#define __ODOM_H

#include "main.h"

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
	
	 // DATA FROM ENCODER（编码器/里程推算相关）
	 float REAL_X;            // 当前 X 坐标（世界/车体坐标，按你的模型定义）
	 float REAL_Y;            // 当前 Y 坐标
	 float REAL_YAW;          // 当前航向角（通常来自 IM948 的 YAW_ANGLE）
	 float REAL_YAW_SET;      // 航向设定值
	 float REAL_YAW_MARK;     // 航向参考/标记值
	 float X_tt;              // X 累积中间量（模型内部积分量）
	 float Y_tt;              // Y 累积中间量（模型内部积分量）
	
} MPU_DATA;


/**
 * @brief 里程推算（odometry）更新
 *
 * 通过电机轴累计角度计算累计里程：
 * - 输入 Theta_m_deg_now：电机轴累计角度（deg）
 * - N：传动比（电机转 N 圈，轮子转 1 圈；直连 N=1）
 * - D：轮子直径（m）
 *
 * 建议以固定周期调用，并使用“累计角度的差分”进行累加。
 *
 * @return 当前累计里程（m）
 */
float odo_update(float Theta_m_deg_now, float N, float D);

/**
 * @brief 清零里程累计
 * @param theta_m_deg_now 当前电机轴累计角度（deg），用于对齐下一次差分
 */
void odo_reset(float theta_m_deg_now);

/* 里程推算周期更新（内部使用 FreeRTOS Tick 做上电预热计时） */
void odo_calculate(void);

#endif
