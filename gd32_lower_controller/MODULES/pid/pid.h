#ifndef __PID_H                          // 头文件防重复包含：如果没定义 __PID_H，则进入
#define __PID_H                          // 定义宏 __PID_H，标记本头文件已被包含

#include "main.h"                       // 工程公共头：通常包含芯片/外设/基础类型等
#include "stdint.h"                     // 标准整型：uint32_t、uint64_t 等（这里用的是字符串 include 形式）
//#include "memory.h"                   // 预留：可能用于 memcpy/memset 的替代实现（当前未启用）
#include "stdlib.h"                     // 标准库：abs、malloc 等（这里主要用到 abs/stdlib 相关）
//#include "bsp_dwt.h"                  // 预留：DWT 计时/时间差计算（本工程目前在 .c 里调用 DWT_GetDeltaT）
//#include "arm_math.h"                 // 预留：CMSIS-DSP（如需更快数学计算可启用）
#include <math.h>                        // 数学库：fabsf 等（PID 堵转检测用了 fabsf）

#ifndef abs                              // 如果系统/库没有提供 abs 宏或函数（或没包含到）
#define abs(x) ((x > 0) ? x : -x)        // 定义 abs 宏：返回 x 的绝对值（注意对浮点/副作用表达式需谨慎）
#endif                                   // 结束 abs 的条件编译

// PID 优化环节使能标志位：用“位标志”表示启用哪些改进；可用位与(&)判断某项是否开启
typedef enum                             // 定义一个枚举类型，用于表示 PID 的改进开关集合
{
    PID_IMPROVE_NONE = 0x00U,                     // 0000 0000：不启用任何改进
    PID_Integral_Limit = 0x01U,                   // 0000 0001：积分限幅
    PID_Derivative_On_Measurement = 0x02U,        // 0000 0010：微分先行（对测量值微分而非误差微分）
    PID_Trapezoid_Intergral = 0x04U,              // 0000 0100：梯形积分（用 (e+e_last)/2 积分）
    PID_Proportional_On_Measurement = 0x08U,      // 0000 1000：比例先行（此版本 .c 中未实现/未使用）
    PID_OutputFilter = 0x10U,                     // 0001 0000：输出低通滤波
    PID_ChangingIntegrationRate = 0x20U,          // 0010 0000：变速积分/积分分离（误差大时减弱积分）
    PID_DerivativeFilter = 0x40U,                 // 0100 0000：微分低通滤波
    PID_ErrorHandle = 0x80U,                      // 1000 0000：异常/堵转检测处理
} PID_Improvement_e;                              // 枚举类型名：PID_Improvement_e

/* PID 报错类型枚举 */                   // 说明：用于描述 PID 检测到的错误类型
typedef enum errorType_e                 // 定义错误类型枚举
{
    PID_ERROR_NONE = 0x00U,              // 无错误
    PID_MOTOR_BLOCKED_ERROR = 0x01U      // 电机堵转错误（输出有、速度/位置无明显变化等）
} ErrorType_e;                           // 枚举类型名：ErrorType_e

typedef struct                           // PID 错误处理结构体：记录错误次数与类型
{
    uint64_t ERRORCount;                 // 错误计数：满足堵转条件的累计次数
    ErrorType_e ERRORType;               // 当前错误类型：无错误/堵转等
} PID_ErrorHandler_t;                    // 结构体类型名：PID_ErrorHandler_t

/* PID结构体 */                          // PID 实例：保存参数、状态量、输出等
typedef struct                           // 定义 PIDInstance 结构体
{
    //---------------------------------- init config block
    // config parameter                 // 以下是“配置参数”（初始化时赋值，运行中通常不变）
    float Kp;                           // 比例系数
    float Ki;                           // 积分系数
    float Kd;                           // 微分系数
    float MaxOut;                       // 输出最大值（限幅）
    float DeadBand;                     // 死区：误差小于此值则认为无需调节

    // improve parameter                // 以下是“改进项参数”（某些改进功能会用到）
    PID_Improvement_e Improve;          // 改进项开关集合（位标志）
    float IntegralLimit;                // 积分限幅：限制 Iout 的最大值
    float CoefA;                        // 变速积分参数 A
    float CoefB;                        // 变速积分参数 B：当 B<|err|<A+B 时按比例衰减积分
    float Output_LPF_RC;                // 输出滤波器 RC：RC=1/ωc（离散实现用 dt/RC）
    float Derivative_LPF_RC;            // 微分滤波器 RC：用于滤除高频噪声

    //-----------------------------------
    // for calculating                  // 以下是“运行状态量”（每次计算都会更新）
    float Measure;                      // 当前测量值（反馈值）
    float Last_Measure;                 // 上一次测量值
    float Err;                          // 当前误差：Ref - Measure
    float Last_Err;                     // 上一次误差
    float Last_ITerm;                   // 上一次积分增量项（ITerm）

    float Pout;                         // 比例输出项
    float Iout;                         // 积分输出累加值
    float Dout;                         // 微分输出项
    float ITerm;                        // 本次积分增量（将累加到 Iout）

    float Output;                       // PID 最终输出
    float Last_Output;                  // 上一次最终输出（用于输出滤波等）
    float Last_Dout;                    // 上一次微分项（用于微分滤波等）

    float Ref;                          // 目标值/设定值

    uint32_t DWT_CNT;                   // 兼容保留：旧版本用于 DWT 计时（当前“无 DWT”版本不使用）
    float dt;                           // 兼容保留：旧版本用于显式 dt（当前“无 DWT”版本不使用）

    PID_ErrorHandler_t ERRORHandler;    // 错误处理器：堵转检测计数/类型
} PIDInstance;                          // 结构体类型名：PIDInstance

/**
 * @brief 初始化PID实例                 // 函数功能：清零 PIDInstance 的运行状态
 * @note  本版本不使用 DWT/dt；Kp/Ki/Kd 等参数需由上层自行赋值
 * @param pid    PID实例指针            // 输入：PIDInstance 指针
 */
void PIDInit(PIDInstance *pid);         // PID 初始化接口声明

/**
 * @brief 计算PID输出                   // 函数功能：根据测量值与目标值计算输出
 *
 * @param pid     PID实例指针           // 输入：PIDInstance 指针
 * @param measure 反馈值               // 输入：测量值（反馈）
 * @param ref     设定值               // 输入：目标值（期望）
 * @return float  PID计算输出          // 输出：本次 PID 输出值
 */
float PIDCalculate(PIDInstance *pid, float measure, float ref); // PID 计算接口声明

#endif                                   // 结束头文件防重复包含
