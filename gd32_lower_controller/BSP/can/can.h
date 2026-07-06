#ifndef __CAN_H
#define __CAN_H

#include "sys.h"
#include "gd32f4xx_can.h"
#include <stdbool.h>

/* 可选：如果启用 FreeRTOS，则 CAN 接收中断会把每一帧都入队，避免只用一个全局缓冲被覆盖 */
typedef struct {
	uint32_t efid;
	uint32_t sfid;
	uint8_t ff;
	uint8_t dlen;
	uint8_t data[8];
} can_rx_frame_t;

void bsp_can_init(void);

typedef struct {
	__IO can_receive_message_struct CAN_RxMsg;
	__IO can_trasnmit_message_struct CAN_TxMsg;

	__IO bool rxFrameFlag;
}CAN_t;

void can_SendCmd(__IO uint8_t *cmd, uint8_t len);

/* CAN 接收帧队列（FreeRTOS 下有效） */
void CAN_RxQueue_Init(void);
bool CAN_RxQueue_TryReceive(can_rx_frame_t *out);

extern __IO CAN_t can; // can.CAN_RxMsg.rx_data[i]

/* Debug: 观察最后一次发送的 CAN 帧（Watch 用） */
extern __IO can_trasnmit_message_struct can_tx_debug_last; // 最后一次发送前组包内容
extern __IO uint8_t can_tx_debug_mailbox;                  // can_message_transmit 返回的邮箱号（或 CAN_NOMAILBOX）
extern __IO uint32_t can_tx_debug_count;                   // 发送帧计数（每发一帧 +1）

extern __IO can_error_enum can_tx_debug_error;             // 最近一次读取到的 CAN 错误类型
extern __IO uint8_t can_tx_debug_tec;                      // TEC: transmit error counter
extern __IO uint8_t can_tx_debug_rec;                      // REC: receive error counter
extern __IO uint32_t can_tx_debug_stat_reg;                // CAN_STAT 寄存器快照
extern __IO uint32_t can_tx_debug_err_reg;                 // CAN_ERR 寄存器快照
extern __IO uint32_t can_tx_debug_tstat_reg;               // CAN_TSTAT 寄存器快照

#endif
