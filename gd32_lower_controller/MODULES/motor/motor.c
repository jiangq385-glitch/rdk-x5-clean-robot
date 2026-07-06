#include "motor.h"



 //电机速度结算
 #define WHEEL_BASE_M        (0.28f)     // 轮距 L：左右轮中心距离(米)
 #define WHEEL_RADIUS_M      (0.05f)     // 轮半径 R(米)

 #define STEPS_PER_REV       (200.0f)    // 1.8°步进电机=200步/圈
 #define MICROSTEP           (64.0f)     // 细分，比如 16
 #define GEAR_RATIO          (1.0f)      // 减速比(电机转 / 轮转)，直连=1

 #define MAX_STEPS_PER_S     (3000u)  // 限速：按你的脉冲上限改
 #define DIR_CW              (0u)
 #define DIR_CCW             (1u)





 //电机速度解算

 vel_cmd_t vel_cmd;

 static float clampf(float x, float lo, float hi)
 {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
 }

 /* 轮线速度 m/s -> 电机 rpm */
 static float wheel_v_to_motor_rpm(float v_mps)
 {
     // wheel_rps = v / (2*pi*R)
     float wheel_rps = v_mps / (2.0f * 3.1415926f * WHEEL_RADIUS_M);
     float motor_rps = wheel_rps * GEAR_RATIO;
     return motor_rps * 60.0f; // rpm
 }

 /* 差速解算并下发 */
 void Differential_kinematics(vel_cmd_t *cmd)
 {
 	float rpm_l;
 	float rpm_r;

     // 1) 车体速度 -> 左右轮线速度
 	cmd->v_l = cmd->vx - 0.5f * WHEEL_BASE_M * cmd->vz;
 	cmd->v_r = cmd->vx + 0.5f * WHEEL_BASE_M * cmd->vz;

     // 2) 线速度 -> 电机 rpm
 	rpm_l = wheel_v_to_motor_rpm(cmd->v_l);
 	rpm_r = wheel_v_to_motor_rpm(cmd->v_r);

     // 4) 限幅
 	rpm_l = clampf(rpm_l, -((float)MAX_STEPS_PER_S), (float)MAX_STEPS_PER_S);
 	rpm_r = clampf(rpm_r, -((float)MAX_STEPS_PER_S), (float)MAX_STEPS_PER_S);

     // 5) 拆方向 + 绝对速度
 	cmd->dir_l = (rpm_l >= 0.0f) ? DIR_CW : DIR_CCW;
 	cmd->dir_r = (rpm_r <= 0.0f) ? DIR_CW : DIR_CCW;

 	cmd->rpm_l = (uint16_t)(fabsf(rpm_l) + 0.5f);
 	cmd->rpm_r = (uint16_t)(fabsf(rpm_r) + 0.5f);

 }


