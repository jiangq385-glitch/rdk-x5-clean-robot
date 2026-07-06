
 #ifndef __MOTOR_H
 #define __MOTOR_H

 #include "main.h"



 typedef struct 
 {
     uint8_t LEFT_ADDR;
     uint8_t RIGHT_ADDR;
     float vx;      // m/s
     float vy;      // m/s (差速可不用)
     float vz;      // rad/s
     uint32_t seq1;  // 更新序号
     uint32_t seq2;
     uint32_t tick; // 接收时刻
     float v_l;
     float v_r;
     uint16_t rpm_l;
     uint16_t rpm_r;
     uint8_t dir_l;
     uint8_t dir_r;

 } vel_cmd_t;
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
 
 //电机速度解算
 extern vel_cmd_t vel_cmd;
 void Differential_kinematics(vel_cmd_t *cmd);

 #endif
