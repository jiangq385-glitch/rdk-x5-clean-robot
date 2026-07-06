#include "odom.h"

/* 里程计算是否通过串口周期输出（0=不输出，仅计算；1=输出） */
#ifndef MOTOR_ODO_TELEMETRY_ENABLE
#define MOTOR_ODO_TELEMETRY_ENABLE 0
#endif

#if MOTOR_ODO_TELEMETRY_ENABLE
#include <string.h>
#include <stdio.h>
#endif



// 输入：Theta_m_deg_now（电机轴累计角度，deg）
// 参数：N（传动比），D（轮径，m）
static float Theta_last = 0.0f;
static float S_total_m = 0.0f;
//步进电机里程计算
float odo_update(float Theta_m_deg_now, float N, float D)
{
    float dTheta = Theta_m_deg_now - Theta_last;
    Theta_last = Theta_m_deg_now;

    float dS = (dTheta / (360.0f * N)) * (3.1415926f * D);
    S_total_m += dS;
    return S_total_m;
}

void odo_reset(float theta_m_deg_now)
{
    Theta_last = theta_m_deg_now;
    S_total_m = 0.0f;
}

//里程计里程计算
MPU_DATA mpu_data;

static void odo_runtime_init_once(void)
{
	mpu_data.cali = 1; // 使能里程计算/数据更新（项目自定义开关）
	mpu_data.vel[0] = 0;
	mpu_data.vel[1] = 0;
	mpu_data.REAL_YAW_SET = 0;
	mpu_data.REAL_YAW_MARK = 0;
}


void odo_calculate(void)
{
	static uint8_t odo_inited = 0;
	static uint8_t warmup_done = 0;
	static TickType_t warmup_start_tick = 0;

#if MOTOR_ODO_TELEMETRY_ENABLE
	static uint32_t add = 0;
#endif

	if (odo_inited == 0) {
		odo_runtime_init_once();
		warmup_start_tick = xTaskGetTickCount();
		warmup_done = 0;

#if MOTOR_ODO_TELEMETRY_ENABLE
		add = 0;
#endif
		odo_inited = 1;
	}

	if(mpu_data.cali == 1){
//					rtU.X_ACCIN  = mpu_data[0].acc_cali[0];
//					rtU.Y_ACCIN  = mpu_data[0].acc_cali[1];

					/* 首次进入先做一次短预热，之后每次调用都直接进入正常计算 */
					if (warmup_done == 0) {
						AS5048_getREGValue(1);
						AS5048_dataUpdate(1);
						// AS5048_getREGValue(2);
						// AS5048_dataUpdate(2);
						mpu_data.vel[0] = 0;
						mpu_data.vel[1] = 0;
						mpu_data.vel[2] = 0;

						if ((xTaskGetTickCount() - warmup_start_tick) < pdMS_TO_TICKS(10)) {
							return;
						}
						warmup_done = 1;
					}

					// 预热完成后进入正常计算（使用 FreeRTOS Tick 计时，不依赖循环频率）
#if MOTOR_ODO_TELEMETRY_ENABLE
						add++;
#endif
          // 编码器：读寄存器 + 更新增量 delta_dis（内部包含回绕处理）
						AS5048_getREGValue(1);
					//	HAL_Delay(1);
						AS5048_dataUpdate(1);	
				//		HAL_Delay(1);
				// 		AS5048_getREGValue(2);
				// //		HAL_Delay(1);
				// 		AS5048_dataUpdate(2);	
				//		HAL_Delay(1);
						
          // 航向角（单位：度，IM_TEST_step 内会做 DEG->rad）
			
			mpu_data.PITCH_ANGLE = Pitch_Angle;
			mpu_data.ROLL_ANGLE  = Roll_Angle;
			mpu_data.YAW_ANGLE   = Yaw_Angle;
			mpu_data.REAL_YAW = Yaw_Angle;// - mpu_data.REAL_YAW_SET + mpu_data.REAL_YAW_MARK ;
		//顺便读取im948的角度数据，虽然目前模型里没用到，但先放在这里了，免得以后忘了哪里读过了
         /*四元素*/
		   mpu_data.quat[0]=quatw;
           mpu_data.quat[1]=quatx;
           mpu_data.quat[2]=quaty;
           mpu_data.quat[3]=quatz;
        /*三轴角速度*/
            mpu_data.gyro[0]=gyrox;
            mpu_data.gyro[1]=gyroy;
            mpu_data.gyro[2]=gyroz;
        /*三轴加速度*/
            mpu_data.acc[0]=accx_nog;
            mpu_data.acc[1]=accy_nog;
            mpu_data.acc[2]=accz_nog;
        /*三轴速度(不一定能用)*/
            mpu_data.vel[0] += mpu_data.acc[0] * 0.01f; // 简单积分，dt=0.01s
            mpu_data.vel[1] += mpu_data.acc[1] * 0.01f;
            mpu_data.vel[2] += mpu_data.acc[2] * 0.01f;
          // Simulink 模型输入：W1/W2 为两路增量（符号由安装方向决定）
            // rtU.W1 = -AS5048s[1].delta_dis;
            rtU.W2 = AS5048s[0].delta_dis;
            rtU.DEG = mpu_data.REAL_YAW;
						
          // 累加模型输出得到里程（注意：此处累加的是“上一次 step 的 rtY”，属于 1 个周期延迟的写法）
            mpu_data.Y_tt += rtY.YOUT ;//*0.0114984;
            mpu_data.X_tt += rtY.XOUT ;//*0.0114984;
          // 比例系数：把 X_tt/Y_tt 的单位换算到实际坐标单位（系数来源于标定/轮径等）
  			mpu_data.REAL_Y = mpu_data.Y_tt * 0.0114984;
			mpu_data.REAL_X = mpu_data.X_tt * 0.0114984;
						// x y yaw
						
						// 处理串口指令帧：允许外部重载坐标/复位
						//Rcv_DealData();
						//这里还没配置


#if MOTOR_ODO_TELEMETRY_ENABLE
						if(add >= 50){
//							if(1==rst_temp){
//								HAL_GPIO_WritePin(RST_CTRL_GPIO_Port,RST_CTRL_Pin,GPIO_PIN_SET);
//								HAL_GPIO_WritePin(RST_CTRL_GPIO_Port,RST_CTRL_Pin,GPIO_PIN_RESET);
//								rst_temp = 0;
//							}
              // 串口输出：bc X Y YAW ROLL（便于上位机/ROS 解析）
              memset(mpu_buff, 0, 64);//bc 握手/数据前缀
							int mpu_len = sprintf(mpu_buff,"bc %f %f %f %f\r\n",mpu_data[0].REAL_X,mpu_data[0].REAL_Y,mpu_data[0].REAL_YAW,mpu_data[0].ROLL_ANGLE);
							HAL_UART_Transmit_DMA(&huart1, (uint8_t *)&mpu_buff, mpu_len);
							//printf("%f %f %f\r\n",mpu_data[0].REAL_X,mpu_data[0].REAL_Y,mpu_data[0].REAL_YAW);
						  add = 0;
						}
#endif
						
					}
                   

        // 模型推进一步：根据本周期 rtU 计算并更新 rtY
        IM_TEST_step();
			 			 
		 }
		


//电机速度解算


