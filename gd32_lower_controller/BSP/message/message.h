#ifndef __MESSAGE_H
#define __MESSAGE_H

#include "sys.h"

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#define FRAME_HEADER 0x55
#define FRAME_LEN    0x07
#define FRAME_TAIL   0xBB
#define FRAME_SIZE   12

// 浣庝笁浣嶏細bit2=x, bit1=y, bit0=z
#define FLAG_X_POS (1u << 2)
#define FLAG_Y_POS (1u << 1)
#define FLAG_Z_POS (1u << 0)

// 消息类型定义
#define TYPE_CMD_VEL    0x01 // ROS -> STM32：目标速度 vx vy wz

// 机械臂协议类型号：关节目标值单位为 rad，发送前按 abs(rad) * 1000 打包，正负号由 flag 表示。
// 机械臂 0：
// 0x21: arm0 的 1、2、3 轴目标
// 0x22: arm0 的 4、5、6 轴目标
// 0x25: arm0 回零命令
// 机械臂 1：
// 0x31: arm1 的 1、2、3 轴目标
// 0x32: arm1 的 4、5、6 轴目标
// 0x35: arm1 回零命令
#define TYPE_ARM0_JOINT_123 0x21 // ROS -> GD32：arm0 1/2/3 轴关节目标
#define TYPE_ARM0_JOINT_456 0x22 // ROS -> GD32：arm0 4/5/6 轴关节目标
#define TYPE_ARM0_HOME      0x25 // ROS -> GD32：arm0 回零命令
#define TYPE_ARM1_JOINT_123 0x31 // ROS -> GD32：arm1 1/2/3 轴关节目标
#define TYPE_ARM1_JOINT_456 0x32 // ROS -> GD32：arm1 4/5/6 轴关节目标
#define TYPE_ARM1_HOME      0x35 // ROS -> GD32：arm1 回零命令

#define TYPE_ODOM_POSE  0x81 // STM32 -> ROS：里程计位姿 x y yaw
#define TYPE_IMU_ACC    0x82 // STM32 -> ROS：ax ay az
#define TYPE_IMU_GYR    0x83 // STM32 -> ROS：gx gy gz
#define TYPE_BAT_MV     0x84 // STM32 -> ROS：电压
#define TYPE_ROBOT_VEL  0x85 // STM32 -> ROS：底盘速度 vx vy wz
#define TYPE_IMU_YAW    0x86 // STM32 -> ROS：IMU偏航角 yaw(rad)

// 机械臂回传状态：GD32 -> ROS
// 约定：协议里回传三个数据，其中 1 0 0 表示任务完成，0 0 0 表示任务未完成。
// 机械臂 0 状态：0x93
// 机械臂 1 状态：0xA3
#define TYPE_ARM0_STATUS 0x93 // GD32 -> ROS：arm0 任务完成状态
#define TYPE_ARM1_STATUS 0xA3 // GD32 -> ROS：arm1 任务完成状态

//激光测距模块
#define TYPE_LIDAR_RANGE 0x50 // STM32 -> ROS：激光测距数据
#define TYPE_LIDAR_STATUS 0x51 // ROS -> STM32：激光测距模块状态

typedef struct
{
    float vx;     // m/s
    float vy;     // m/s
    float vz;     // rad/s(閹存牔缍橀惃鍕礋娴ｏ拷)
    uint8_t flag;
} speed_frame_t;

typedef struct
{
    uint8_t type_id;
    float x;
    float y;
    float z;
    uint8_t flag;
} host_frame_t;

void set_message(uint8_t type_id,float vx, float vy,float vz,uint8_t flag, uint8_t out[12]);
bool feed_host_frame(uint8_t b, host_frame_t *out);
void host_frame_apply_sign(host_frame_t *f);
bool feed_speed_frame(uint8_t b, speed_frame_t *out);
void speed_val (speed_frame_t *f);

#endif

