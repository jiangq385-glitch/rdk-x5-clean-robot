#include "can.h"             // CAN 驱动头文件（包含 GD32 CAN 类型定义）
#include <stddef.h>           // NULL 定义（用于参数判空）

#if SYSTEM_SUPPORT_OS
#include "FreeRTOS.h"
#include "queue.h"
#endif

int Can_Receive_Count;        // CAN 接收计数（当前文件未使用，仅保留）
int data[8];                  // 8 字节临时数据缓冲（当前文件未使用，仅保留）
/**
	* @brief   初始化CAN
	* @param   无
	* @retval  无
	*/
void bsp_can_init(void)       // CAN 外设初始化：PA11(RX)/PA12(TX) + CAN0 + FIFO0 RX0 中断 + 500kbps(常见 CAN_CLK=42MHz 时)
{
	can_parameter_struct can_parameter;          // CAN 初始化参数结构体
	can_filter_parameter_struct can_filter;      // CAN 滤波器参数结构体

	can_struct_para_init(CAN_INIT_STRUCT, &can_parameter);      // 参数结构体默认值初始化
	can_struct_para_init(CAN_FILTER_STRUCT, &can_filter);       // 滤波器结构体默认值初始化
	/* GPIO/RCU: PA11(RX), PA12(TX) -> CAN0 */
	rcu_periph_clock_enable(RCU_GPIOA);          // 使能 GPIOA 时钟（PA11/PA12）
	rcu_periph_clock_enable(RCU_CAN0);           // 使能 CAN0 时钟

	gpio_af_set(GPIOA, GPIO_AF_9, GPIO_PIN_11);                                  // PA11 复用到 AF9（CAN）
	gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_11);             // PA11 复用模式、无上下拉
	gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_11);// PA11 推挽、50MHz

	gpio_af_set(GPIOA, GPIO_AF_9, GPIO_PIN_12);                                  // PA12 复用到 AF9（CAN）
	gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_12);             // PA12 复用模式、无上下拉
	gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_12);// PA12 推挽、50MHz

	/* NVIC: CAN0 FIFO0 RX interrupt */
	nvic_irq_enable(CAN0_RX0_IRQn, 6, 0);             // 使能 CAN0 FIFO0 接收中断：抢占优先级6，子优先级0

	/* deinit + init CAN0, 500kbps（当前这组时序参数） */
	can_deinit(CAN0);                                 // 复位 CAN0 外设寄存器到缺省状态
	can_parameter.time_triggered = DISABLE;           // 禁用 TTCM：时间触发通信
	can_parameter.auto_bus_off_recovery = DISABLE;    // 禁用 ABOM：自动离线恢复
	can_parameter.auto_wake_up = DISABLE;             // 禁用 AWUM：自动唤醒
	can_parameter.no_auto_retrans = ENABLE;           // NART=ENABLE：禁止自动重传
	can_parameter.rec_fifo_overwrite = ENABLE;        // RFLM=DISABLE 等价：FIFO 满时新报文覆盖旧报文
	can_parameter.trans_fifo_order = DISABLE;         // 发送优先级按标识符（非按请求顺序）
	can_parameter.working_mode = CAN_NORMAL_MODE;     // 正常模式（非回环/静默）
	can_parameter.resync_jump_width = CAN_BT_SJW_1TQ; // SJW = 1TQ
	can_parameter.time_segment_1 = CAN_BT_BS1_4TQ;    // BS1 = 4TQ
	can_parameter.time_segment_2 = CAN_BT_BS2_1TQ;    // BS2 = 1TQ
	can_parameter.prescaler = 14;                     // 分频系数：常见 CAN_CLK=42MHz 时得到 500kbps（42M/(14*(1+4+1))）
	can_init(CAN0, &can_parameter);             // 初始化 CAN0（忽略返回值）

	/* filter: 不过滤任何帧，挂到 FIFO0 */
	can_filter.filter_number = 0;                     // 选择滤波器编号（CAN0:0-13）
	can_filter.filter_mode = CAN_FILTERMODE_MASK;     // 掩码模式（MASK）
	can_filter.filter_bits = CAN_FILTERBITS_32BIT;    // 32 位滤波
	can_filter.filter_list_high = 0x0000;            // ID 高 16 位（mask=0 时该值无影响）
	can_filter.filter_list_low = 0x0000;             // ID 低 16 位（mask=0 时该值无影响）
	can_filter.filter_mask_high = 0x0000;            // MASK 高 16 位（全 0 = 全放行）
	can_filter.filter_mask_low = 0x0000;             // MASK 低 16 位（全 0 = 全放行）
	can_filter.filter_fifo_number = CAN_FIFO0;        // 关联到 FIFO0（对应 RX0 中断）
	can_filter.filter_enable = ENABLE;                // 使能该滤波器
	can_filter_init(&can_filter);                     // 应用滤波器配置

	/* enable CAN receive FIFO0 not empty interrupt */
	can_interrupt_enable(CAN0, CAN_INT_RFNE0);         // 使能 FIFO0 非空中断

	/* 创建 CAN 接收队列（FreeRTOS 下有效） */
	CAN_RxQueue_Init();

}


/**
  * @brief  获取CAN接收消息中的数据
  * @param  can 指向CAN_t结构体的指针
  * @param  data 用于存储数据的缓冲区(至少8字节)
  * @retval 无
  */
void CAN_GetRxData(CAN_t *can, uint8_t *data)
{
	if (can == NULL || data == NULL) {                // 判空：结构体指针或输出缓冲为空就直接返回
		return;                                         // 避免空指针访问
	}

	// 复制接收结构体里的 8 字节数据到用户缓冲区
	for (int i = 0; i < 8; i++) {                     // CAN 数据域固定最大 8 字节
		data[i] = can->CAN_RxMsg.rx_data[i];            // 逐字节拷贝
	}
}

__IO CAN_t can = {0};                                // 全局 CAN 收发对象（中断与任务共享）

#if SYSTEM_SUPPORT_OS
static QueueHandle_t s_can_rx_queue = NULL;
#endif

void CAN_RxQueue_Init(void)
{
#if SYSTEM_SUPPORT_OS
	if (s_can_rx_queue == NULL) {
		/* 16 帧缓存，足够应对短时间的回包/ACK 突发 */
		s_can_rx_queue = xQueueCreate(32, sizeof(can_rx_frame_t));
	}
#endif
}

bool CAN_RxQueue_TryReceive(can_rx_frame_t *out)
{
#if SYSTEM_SUPPORT_OS
	if (s_can_rx_queue == NULL || out == NULL) {
		return false;
	}
	return (xQueueReceive(s_can_rx_queue, out, 0) == pdTRUE);
#else
	(void)out;
	return false;
#endif
}
//void CAN2_TX_IRQHandler(void)
//{
//	printf("CAN2_TX_IRQn");
//}
/**
	* @brief   CAN1_RX0接收中断
	* @param   无
	* @retval  无
	*/
void CAN0_RX0_IRQHandler(void)
{
	//LED0_Set(0);                                                               // 调试用：点亮/翻转 LED
	//Can_Receive_Count += 1;                                                    // 调试用：统计接收次数
	// 接收一包数据（从 FIFO0 取出一帧）
	can_message_receive(CAN0, CAN_FIFO0, (can_receive_message_struct *)(&can.CAN_RxMsg)); // 把接收帧存到 can.CAN_RxMsg
	//printf("1");                                                             // 调试用：打印
	// 一帧数据接收完成，置位帧标志位（供任务/主循环轮询）
	can.rxFrameFlag = true;                                                     // 通知“已收到新帧”

#if SYSTEM_SUPPORT_OS
	/* 入队：上层可逐帧解析，避免单个全局缓冲被覆盖 */
	if (s_can_rx_queue != NULL) {
		can_rx_frame_t frame;
		frame.efid = can.CAN_RxMsg.rx_efid;
		frame.sfid = can.CAN_RxMsg.rx_sfid;
		frame.ff   = can.CAN_RxMsg.rx_ff;
		frame.dlen = can.CAN_RxMsg.rx_dlen;
		for (uint8_t i = 0; i < 8; i++) {
			frame.data[i] = can.CAN_RxMsg.rx_data[i];
		}
		BaseType_t hpw = pdFALSE;
		(void)xQueueSendFromISR(s_can_rx_queue, &frame, &hpw);
		portYIELD_FROM_ISR(hpw);
	}
#endif

}

/**
	* @brief   CAN发送多个字节
	* @param   无
	* @retval  无
	*/
void can_SendCmd(__IO uint8_t *cmd, uint8_t len)
{
	__IO uint8_t i = 0, j = 0, k = 0, l = 0, packNum = 0; // i:数据偏移 j:有效长度 k:剩余长度 l:循环 packNum:分包号

	// 除去“ID地址(cmd[0]) + 功能码(cmd[1])”后的数据长度
	j = len - 2;                                           // 真实负载字节数（从 cmd[2] 开始）

	// 发送数据：按 1 个功能码 + 最多 7 个负载字节进行分包（每帧最多 8 字节）
	while(i < j)                                           // 直到负载发送完
	{
		// 计算本次还剩多少负载字节
		k = j - i;                                           // 剩余负载长度

		// 填充发送结构体（每一帧都重新组包）
		can.CAN_TxMsg.tx_sfid = 0x00;                        // 标准帧 ID（扩展帧下通常不用，这里置 0）
		can.CAN_TxMsg.tx_efid = ((uint32_t)cmd[0] << 8) | (uint32_t)packNum; // 扩展帧 ID：高字节=设备ID，低字节=分包号
		can.CAN_TxMsg.tx_ff = CAN_FF_EXTENDED;               // 扩展帧格式
		can.CAN_TxMsg.tx_ft = CAN_FT_DATA;                   // 数据帧（非远程帧）
		for (l = 0; l < 8; l++) {                            // 先清空 8 字节数据域
			can.CAN_TxMsg.tx_data[l] = 0;
		}
		can.CAN_TxMsg.tx_data[0] = cmd[1];                   // 第 0 字节放功能码

		// 小于 8 字节负载：本帧可一次发完（但要给功能码留 1 字节）
		if(k < 8)                                            // 这里 k 是“剩余负载字节数”
		{
			for(l=0; l < k; l++,i++) {                         // 把剩余负载搬到 tx_data[1..]
				can.CAN_TxMsg.tx_data[l + 1] = cmd[i + 2];
			}
			can.CAN_TxMsg.tx_dlen = k + 1;                      // DLC=功能码1字节 + 负载k字节
		}
		// 大于等于 8 字节负载：分包发送，每帧携带 1(功能码) + 7(负载)
		else
		{
			for(l=0; l < 7; l++,i++) {                         // 每帧最多再塞 7 字节负载
				can.CAN_TxMsg.tx_data[l + 1] = cmd[i + 2];
			}
			can.CAN_TxMsg.tx_dlen = 8;                          // DLC=8（满帧）
		}
		

		// 发送该帧数据
		can_message_transmit(CAN0, (can_trasnmit_message_struct *)(&can.CAN_TxMsg)); // 通过 CAN0 发送
//		uint8_t status;
//		if (status == CAN_TxStatus_NoMailBox) 
//		{
//			printf("CAN发送失败：无空闲邮箱\n");
//		}
		// 记录发送的第几包（用于扩展 ID 的低字节）
		++packNum;                                          // 下一帧分包号 +1
//		printf("packNum = %d\n",packNum);
	}
}
