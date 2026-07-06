#include "encoder.h"
#include <math.h>

/*
 * ENCL(0x31) 回包（说明书）：
 *   data: [addr][0x31][hi][lo][0x6B]
 * ENCL 为 16bit 单圈计数，存在回绕。
 */

// ============ 底盘参数（可根据实际情况修改） ============
#define WHEEL_RADIUS         0.050f    // 轮子半径 (m)
#define WHEEL_BASE           0.3f       // 两轮中心距 (m)
#define ENCODER_RESOLUTION   65535.0f    // 编码器每圈脉冲数（你的电机编码器）
#define GEAR_RATIO           1.0f      // 电机减速比
#define M_PI  3.14
// ============ 里程计状态 ============
static ChassisState chassis = {0};
float sb;
float sbb;
float sbbb;
float sc;
float scc;	
// ============ 速度计算用的增量缓存 ============
static int32_t last_delta[2] = {0};

volatile uint32_t encoder_debug_poll_count = 0;
volatile uint32_t encoder_debug_timeout_count = 0;
volatile uint32_t encoder_debug_decode_fail_count = 0;
volatile uint32_t encoder_debug_rx_count = 0;
volatile uint8_t encoder_debug_last_addr = 0;
volatile uint16_t encoder_debug_last_encl[Encoder_NUMBER + 1] = {0};
volatile int32_t encoder_debug_last_delta[Encoder_NUMBER + 1] = {0};
volatile uint8_t encoder_debug_last_valid[Encoder_NUMBER + 1] = {0};

// ============ 编码器解析相关 ============
#define ENCL_FUNC_CODE          0x31u
#define ENCL_MODULO             65536
#define ENCL_WRAP_THRESHOLD     52428

ENCl ENCLs[Encoder_NUMBER];

/* 最新一次解析到的 ENCL（按电机地址索引：1..Encoder_NUMBER） */
static volatile uint16_t s_encl_latest[Encoder_NUMBER + 1];
static volatile uint8_t s_encl_valid[Encoder_NUMBER + 1];

static bool Encoder_TryDecodeEncl(const can_rx_frame_t *frame, uint8_t *outAddr, uint16_t *outEncl)
{
	uint8_t addr = 0;
	uint8_t hi = 0;
	uint8_t lo = 0;

	if (frame == NULL || outAddr == NULL || outEncl == NULL) {
		return false;
	}

	/* A) 说明书格式：addr 在 data[0] */
	if (frame->dlen >= 5 && frame->data[1] == ENCL_FUNC_CODE) {
		addr = frame->data[0];
		hi = frame->data[2];
		lo = frame->data[3];
	} else if (frame->dlen >= 4 && frame->data[0] == ENCL_FUNC_CODE) {
		/* B) 兼容：addr 可能放在扩展ID高字节，data[0] 直接是功能码 */
		addr = (uint8_t)((frame->efid >> 8) & 0xFFu);
		hi = frame->data[1];
		lo = frame->data[2];
	} else {
		return false;
	}

	if (addr == 0 || addr > Encoder_NUMBER) {
		return false;
	}

	*outAddr = addr;
	*outEncl = (uint16_t)(((uint16_t)hi << 8) | (uint16_t)lo);
	return true;
}

void Encoder_Init(uint8_t Encl_id)
{
	if (Encl_id == 0 || Encl_id > Encoder_NUMBER) {
		return;
	}

	ENCl *encoder = ENCLs + Encl_id - 1;
	encoder->ENCODER_ID = Encl_id;
	encoder->angle = 0;
	encoder->last_angle = 0;
	encoder->total_angle = 0;
	encoder->cirle = 0;
	encoder->delta_dis = 0;

	s_encl_latest[Encl_id] = 0;
	s_encl_valid[Encl_id] = 0;
	encoder_debug_last_encl[Encl_id] = 0;
	encoder_debug_last_delta[Encl_id] = 0;
	encoder_debug_last_valid[Encl_id] = 0;
}

void Encoder_PollAll(uint32_t timeout_ms)
{
	encoder_debug_poll_count++;
	TickType_t startTick = xTaskGetTickCount();
	TickType_t timeoutTicks = pdMS_TO_TICKS(timeout_ms);
	can_rx_frame_t frame;
	uint8_t addr;
	uint16_t encl;
	uint8_t all_ok = 0;
	uint8_t id;

	/* 清掉本轮“新数据”标志：确保等待的是刚发起读取后的回包 */
	for (id = 1; id <= Encoder_NUMBER; id++) {
		s_encl_valid[id] = 0;
	}

	while ((xTaskGetTickCount() - startTick) <= timeoutTicks) {
		if (CAN_RxQueue_TryReceive(&frame)) {
			if (Encoder_TryDecodeEncl(&frame, &addr, &encl)) {
				s_encl_latest[addr] = encl;
				s_encl_valid[addr] = 1;
				encoder_debug_rx_count++;
				encoder_debug_last_addr = addr;
				encoder_debug_last_encl[addr] = encl;
				encoder_debug_last_valid[addr] = 1;
			} else {
				encoder_debug_decode_fail_count++;
			}
		} else {
			vTaskDelay(1);
		}

		/* 已收齐 1..Encoder_NUMBER 的 ENCL 回包则提前退出（你当前是 1 和 2） */
		all_ok = 1;
		for (id = 1; id <= Encoder_NUMBER; id++) {
			if (!s_encl_valid[id]) {
				all_ok = 0;
				break;
			}
		}
		if (all_ok) {
			break;
		}
	}

	if (!all_ok) {
		encoder_debug_timeout_count++;
	}
}

static bool Encoder_GetLatest(uint8_t Encl_id, uint16_t *outEncl)
{
	if (Encl_id == 0 || Encl_id > Encoder_NUMBER || outEncl == NULL) {
		return false;
	}
	if (!s_encl_valid[Encl_id]) {
		return false;
	}
	*outEncl = s_encl_latest[Encl_id];
	s_encl_valid[Encl_id] = 0; /* 消费掉本次更新 */
	return true;
}

void Encoder_dataUpdate(uint8_t Encl_id)
{
	if (Encl_id == 0 || Encl_id > Encoder_NUMBER) {
		return;
	}

	ENCl *encoder = ENCLs + Encl_id - 1;
	uint16_t angle_u16;
	int32_t diff;
	int32_t rawDiff;

	if (!Encoder_GetLatest(Encl_id, &angle_u16)) {
		/* 没收到新数据：本周期增量按 0 处理 */
		encoder->delta_dis = 0;
		encoder_debug_last_delta[Encl_id] = 0;
		encoder_debug_last_valid[Encl_id] = 0;
		return;
	}

	encoder->angle = (int)angle_u16;
	rawDiff = (int32_t)encoder->angle - (int32_t)encoder->last_angle;
	diff = rawDiff;

	// 如果右轮安装反向（硬件倒装），对右轮增量取反以修正方向
	if (Encl_id == 1) {
		diff = -diff;
	}

	/* 16bit 回绕处理：把差值映射到 (-32768, +32767) */
	if (diff > ENCL_WRAP_THRESHOLD) {
		diff -= ENCL_MODULO;
		encoder->cirle--;
	} else if (diff < -ENCL_WRAP_THRESHOLD) {
		diff += ENCL_MODULO;
		encoder->cirle++;
	}

	encoder->delta_dis = (int)diff;
	encoder->total_angle += (int)diff;
	encoder->last_angle = encoder->angle;
	encoder_debug_last_delta[Encl_id] = diff;
	encoder_debug_last_valid[Encl_id] = 1;
	
}
// ============ 里程计算（每50ms调用一次） ============
void update_odometry(void)
{
    // 编码器增量 -> 轮速 -> 里程积分

    // 每圈脉冲数 = ENCODER_RESOLUTION * GEAR_RATIO
    float pulses_per_rev = ENCODER_RESOLUTION * GEAR_RATIO;
    // 每个脉冲对应的路程 (m)
    float dist_per_pulse = (2.0f * (float)M_PI * WHEEL_RADIUS) / pulses_per_rev;

    // 固定 50ms 采样周期
    float dt = 0.02f;

    // 左轮速度 (m/s)
    chassis.v_left  = ((float)ENCLs[1].delta_dis * dist_per_pulse) / dt;
    // 右轮速度 (m/s)
    chassis.v_right = ((float)ENCLs[0].delta_dis * dist_per_pulse) / dt;

    // 差速底盘运动学
    chassis.v_linear  = (chassis.v_left + chassis.v_right) * 0.5f;
    chassis.v_angular = (chassis.v_right - chassis.v_left) / WHEEL_BASE;

    // 积分更新位姿
    chassis.theta += chassis.v_angular * dt;

    // 角度归一化到 [-π, π]
    if (chassis.theta > (float)M_PI) {
        chassis.theta -= 2.0f * (float)M_PI;
    } else if (chassis.theta < -(float)M_PI) {
        chassis.theta += 2.0f * (float)M_PI;
    }

    chassis.x += chassis.v_linear * cosf(chassis.theta) * dt;
    chassis.y += chassis.v_linear * sinf(chassis.theta) * dt;
	sb=chassis.v_linear;
	sbb=chassis.v_angular;
	sbbb=chassis.theta;
	sc=chassis.v_left;
	scc=chassis.v_right;
}

// ============ 获取里程计状态 ============
ChassisState* get_chassis_state(void)
{
    return &chassis;
}

void odom_read_task(void)
{

	Emm_V5_Read_Sys_Params(1, S_ENCL);
            Emm_V5_Read_Sys_Params(2, S_ENCL);
            Encoder_PollAll(20);
            Encoder_dataUpdate(1);
            Encoder_dataUpdate(2);
            update_odometry();
}
