#include "exchange.h"



IMU_DATA imu_data;
void exchange_task(void)
{
    // 这里可以放一些周期性的交换/通信逻辑，比如：
    // - 定时发送状态帧（里程计、IMU、编码器等）
    #define DEG_TO_RAD 0.0174532925f
    // - 处理接收到的命令帧
    // - 监控通信状态，进行重连或错误处理

        // 示例：每 35ms 发送一次状态帧
        uint8_t imuacc_frame[12];
        uint8_t imuyr_frame[12];
        uint8_t imuyaw_frame[12];
        uint8_t bat_frame[12];
        uint8_t vel_frame[12];
        uint8_t odom_frame[12];
        float imu_yaw_rad = imu_data.YAW_ANGLE * DEG_TO_RAD;
        ChassisState* chassis = get_chassis_state();
        set_message(TYPE_ROBOT_VEL, chassis->v_linear, 0, chassis->v_angular, 0, vel_frame);
        set_message(TYPE_ODOM_POSE, chassis->x, chassis->y, chassis->theta, 0, odom_frame);//x、y轴的位置和航向角
        set_message(TYPE_IMU_ACC, imu_data.acc[0], imu_data.acc[1], imu_data.acc[2], 0, imuacc_frame);
        set_message(TYPE_IMU_GYR, imu_data.gyro[0], imu_data.gyro[1], imu_data.gyro[2], 0, imuyr_frame);
        set_message(TYPE_IMU_YAW, imu_yaw_rad, 0, 0, 0, imuyaw_frame);
        // set_message(TYPE_BAT_MV, 0, 0, 0, 0, bat_frame); // 电压可以放在 vx 字段，其他字段留空

        UART_SendBytes(vel_frame, 12); // 假设有 uart_send 函数
        vTaskDelay(5); // 延时 5ms
        UART_SendBytes(odom_frame, 12); // 假设有 uart_send 函数
        vTaskDelay(5); // 延时 5ms
        UART_SendBytes(imuacc_frame, 12); // 假设有 uart_send 函数
        vTaskDelay(5); // 延时 5ms
        UART_SendBytes(imuyr_frame, 12); // 假设有 uart_send 函数
        vTaskDelay(5); // 延时 5ms
        UART_SendBytes(imuyaw_frame, 12); // 假设有 uart_send 函数
        // UART_SendBytes(bat_frame, 12); // 假设有 uart_send 函数

        vTaskDelay(20); // 延时 50ms

   //5.6电压暂时不传
}

float sa,saa,saaa;
//实时读取imu数据
void Read_imu_data(void)
{


    // 航向角（单位：度，IM_TEST_step 内会做 DEG->rad）
			
			imu_data.PITCH_ANGLE = Pitch_Angle;
			imu_data.ROLL_ANGLE  = Roll_Angle;
			imu_data.YAW_ANGLE   = Yaw_Angle;
			imu_data.REAL_YAW = Yaw_Angle;// - imu_data.REAL_YAW_SET + imu_data.REAL_YAW_MARK ;
		//顺便读取im948的角度数据，虽然目前模型里没用到，但先放在这里了，免得以后忘了哪里读过了
         /*四元素*/
		   imu_data.quat[0]=quatw;
           imu_data.quat[1]=quatx;
           imu_data.quat[2]=quaty;
           imu_data.quat[3]=quatz;
        /*三轴角速度*/
            imu_data.gyro[0]=gyrox;
            imu_data.gyro[1]=gyroy;
            imu_data.gyro[2]=gyroz;
        /*三轴加速度*/
            imu_data.acc[0]=accx_nog;
            imu_data.acc[1]=accy_nog;
            imu_data.acc[2]=accz_nog;
        /*三轴速度(不一定能用)*/
            imu_data.vel[0] += imu_data.acc[0] * 0.01f; // 简单积分，dt=0.01s
            imu_data.vel[1] += imu_data.acc[1] * 0.01f;
            imu_data.vel[2] += imu_data.acc[2] * 0.01f;
            sa=imu_data.gyro[0];
			saaa=imu_data.gyro[1];
            saa=imu_data.gyro[2];
            
}
