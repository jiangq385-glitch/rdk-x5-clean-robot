
#include "mpu_cal.h"
 MPU_DATA mpu_data[4];
/*void DATARELOAD(uint8_t * arr){
  /*
   * 外部下发“位置重载”协议解析：
   * - arr[0]  : header
   * - arr[1..2]: X（16bit，低字节在前），这里按有符号数解析
   * - arr[3..4]: Y（16bit，低字节在前），这里按有符号数解析
   * - arr[5..6]: yaw 相关（当前被注释未使用）
   * - arr[7]  : footer
   *
   * 解析完成后会更新：
   * - mpu_data[0].REAL_X/REAL_Y
   * - 同步更新 mpu_data[0].X_tt/Y_tt（用 0.014373 做比例换算回内部累计单位）
   */
//		uint16_t temp[2] = {0};
//		
//		if(arr[1]==0x00&&arr[2]==0x00){
//			mpu_data[0].REAL_X = 0;
//		}
//		else if((arr[2]&0x80)==0x80)//璐熸暟
//		{
//      /* 16bit 有符号数：这里用“取反+1”的方式还原负数幅值（沿用原实现） */
//			temp[0] = arr[2];
//			temp[0] = temp[0] << 8;
//			temp[0] += arr[1];
//			temp[0] -= 1;
//			temp[0] = ~temp[0];
//			mpu_data[0].REAL_X = 0-temp[0];
//		}else{
//			mpu_data[0].REAL_X = (arr[1] | arr[2] << 8);
//		}
//		
//		if(arr[3]==0x00&&arr[4]==0x00){
//			mpu_data[0].REAL_Y = 0;
//		}
//		else if((arr[4]&0x80)==0x80)//璐熸暟
//		{
//      /* 16bit 有符号数负值解析（沿用原实现） */
//			temp[1] = arr[4];
//			temp[1] = temp[1] << 8;
//			temp[1] += arr[3];
//			temp[1] -= 1;
//			temp[1] = ~temp[1];
//			mpu_data[0].REAL_Y = 0-temp[1];
//		}else{
//			mpu_data[0].REAL_Y = (arr[3] | arr[4] << 8);
//		}
//		
////		if(arr[5]==0x00&&arr[6]==0x00){
////			mpu_data[0].REAL_YAW_MARK = 0;
////		}
////		else if((arr[6]&0x80)==0x80)//璐熸暟
////		{
////			temp[2] = arr[6];
////			temp[2] = temp[2] << 8;
////			temp[2] += arr[5];
////			temp[2] -= 1;
////			temp[2] = ~temp[2];
////			mpu_data[0].REAL_YAW_MARK = 0-temp[2];
////		}else{
////			mpu_data[0].REAL_YAW_MARK = (arr[5] | arr[6] << 8);
////		}
//		
////    mpu_data[0].REAL_X = (arr[1] | arr[2] << 8);
////    mpu_data[0].REAL_Y = (arr[3] | arr[4] << 8);
//    mpu_data[0].X_tt  = mpu_data[0].REAL_X / 0.014373;
//    mpu_data[0].Y_tt  = mpu_data[0].REAL_Y / 0.014373;

////     mpu_data[0].REAL_YAW_MARK = (arr[5] | arr[6] << 8);
////     mpu_data[0].REAL_YAW_SET = mpu_data[0].YAW_ANGLE;

//}*/
