#ifndef __ENCODER_H
#define __ENCODER_H
#include "main.h"
 
#define Encoder_NUMBER 2  
typedef struct {
  
	
	int ENCODER_ID;
	
  int32_t    angle;
	int32_t    last_angle; //  cc_direction
	int32_t    total_angle;
	int32_t    cirle;
	int32_t    delta_dis;

} ENCl;
typedef struct {
    float x;           // 位置 x (米)
    float y;           // 位置 y (米)
    float theta;       // 航向角 (弧度)
    float v_left;      // 左轮速度 (m/s)
    float v_right;     // 右轮速度 (m/s)
    float v_linear;    // 底盘线速度 (m/s)
    float v_angular;   // 底盘角速度 (rad/s)
} ChassisState;

void Encoder_Init(uint8_t Encl_id);
void Encoder_dataUpdate(uint8_t Encl_id);
void Encoder_PollAll(uint32_t timeout_ms);
void update_odometry(void);
ChassisState* get_chassis_state(void);
void odom_read_task(void);
extern ENCl ENCLs[Encoder_NUMBER];

extern volatile uint32_t encoder_debug_poll_count;
extern volatile uint32_t encoder_debug_timeout_count;
extern volatile uint32_t encoder_debug_decode_fail_count;
extern volatile uint32_t encoder_debug_rx_count;
extern volatile uint8_t encoder_debug_last_addr;
extern volatile uint16_t encoder_debug_last_encl[Encoder_NUMBER + 1];
extern volatile int32_t encoder_debug_last_delta[Encoder_NUMBER + 1];
extern volatile uint8_t encoder_debug_last_valid[Encoder_NUMBER + 1];



#endif
