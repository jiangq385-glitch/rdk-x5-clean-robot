#include "main.h"
/**
 * @file controller.c
 * @author wanghongxi
 * @author modified by neozng
 * @brief  PID控制器定义
 * @version beta
 * @date 2022-11-01
 *
 * @copyrightCopyright (c) 2022 HNU YueLu EC all rights reserved
 */
#include "pid.h"
//#include "memory.h"

/* ----------------------------下面是pid优化环节的实现---------------------------- */

// 梯形积分
static void f_Trapezoid_Intergral(PIDInstance *pid)
{
    // 计算梯形的面积,(上底+下底)*高/2
    // 无 dt 版本：将积分“增量”从 e[k] 改为 (e[k] + e[k-1]) / 2
    // Ki 在主公式里统一相乘，这里只处理误差形状
    pid->ITerm = (pid->Err + pid->Last_Err) / 2.0f;
}

// 变速积分(误差小时积分作用更强)
static void f_Changing_Integration_Rate(PIDInstance *pid)
{
    if (pid->Err * pid->Iout > 0)
    {
        // 积分呈累积趋势
        if (abs(pid->Err) <= pid->CoefB)
            return; // Full integral
        if (abs(pid->Err) <= (pid->CoefA + pid->CoefB))
            pid->ITerm *= (pid->CoefA - abs(pid->Err) + pid->CoefB) / pid->CoefA;
        else // 最大阈值,不使用积分
            pid->ITerm = 0;
    }
}

static void f_Integral_Limit(PIDInstance *pid)
{
    static float temp_Output, temp_Iout;
    // Iout 在无 dt 版本中是“误差累计和”，积分输出为 Ki * Iout
    temp_Iout = pid->Iout + pid->ITerm;
    temp_Output = pid->Pout + pid->Ki * temp_Iout + pid->Kd * pid->Dout;
    if (abs(temp_Output) > pid->MaxOut)
    {
        if (pid->Err * pid->Iout > 0) // 积分却还在累积
        {
            pid->ITerm = 0; // 当前积分项置零
        }
    }

    if (temp_Iout > pid->IntegralLimit)
    {
        pid->ITerm = 0;
        pid->Iout = pid->IntegralLimit;
    }
    if (temp_Iout < -pid->IntegralLimit)
    {
        pid->ITerm = 0;
        pid->Iout = -pid->IntegralLimit;
    }
}

// 微分先行(仅使用反馈值而不计参考输入的微分)
static void f_Derivative_On_Measurement(PIDInstance *pid)
{
    // 无 dt 版本：用测量值差分近似微分（Kd 在主公式里统一相乘）
    pid->Dout = (pid->Last_Measure - pid->Measure);
}

// 微分滤波(采集微分时,滤除高频噪声)
static void f_Derivative_Filter(PIDInstance *pid)
{
    // 无 dt 版本：等价于取采样周期 dt=1 的一阶低通形式
    const float denom = (pid->Derivative_LPF_RC + 1.0f);
    if (denom > 0.0f)
    {
        pid->Dout = pid->Dout / denom + pid->Last_Dout * (pid->Derivative_LPF_RC / denom);
    }
}

// 输出滤波
static void f_Output_Filter(PIDInstance *pid)
{
    // 无 dt 版本：等价于取采样周期 dt=1 的一阶低通形式
    const float denom = (pid->Output_LPF_RC + 1.0f);
    if (denom > 0.0f)
    {
        pid->Output = pid->Output / denom + pid->Last_Output * (pid->Output_LPF_RC / denom);
    }
}

// 输出限幅
static void f_Output_Limit(PIDInstance *pid)
{
    if (pid->Output > pid->MaxOut)
    {
        pid->Output = pid->MaxOut;
    }
    if (pid->Output < -(pid->MaxOut))
    {
        pid->Output = -(pid->MaxOut);
    }
}

// 电机堵转检测
static void f_PID_ErrorHandle(PIDInstance *pid)
{
    /*Motor Blocked Handle*/
    if (fabsf(pid->Output) < pid->MaxOut * 0.001f || fabsf(pid->Ref) < 0.0001f)
        return;

    if ((fabsf(pid->Ref - pid->Measure) / fabsf(pid->Ref)) > 0.95f)
    {
        // Motor blocked counting
        pid->ERRORHandler.ERRORCount++;
    }
    else
    {
        pid->ERRORHandler.ERRORCount = 0;
    }

    if (pid->ERRORHandler.ERRORCount > 500)
    {
        // Motor blocked over 1000times
        pid->ERRORHandler.ERRORType = PID_MOTOR_BLOCKED_ERROR;
    }
}

/* ---------------------------下面是PID的外部算法接口--------------------------- */

/**
 * @brief 初始化PID,设置参数和启用的优化环节,将其他数据置零
 *
 * @param pid    PID实例
 * @param config PID初始化设置
 */
void PIDInit(PIDInstance *pid)
{
    // config的数据和pid的部分数据是连续且相同的的,所以可以直接用memcpy
    // @todo: 不建议这样做,可扩展性差,不知道的开发者可能会误以为pid和config是同一个结构体
    // 后续修改为逐个赋值
    memset(pid, 0, sizeof(PIDInstance));

    /* 以下是逐个赋值的版本,虽然麻烦但更清晰,更不容易出问题
    pid->Kp = config->Kp;          
    pid->Ki = config->Ki;
    pid->Kd = config->Kd;
    pid->MaxOut = config->MaxOut;
    pid->DeadBand = config->DeadBand;
    pid->Improve = config->Improve;
    pid->IntegralLimit = config->IntegralLimit;
    pid->CoefA = config->CoefA;
    pid->CoefB = config->CoefB;
    pid->Output_LPF_RC = config->Output_LPF_RC;
    pid->Derivative_LPF_RC = config->Derivative_LPF_RC;
    */
}

/**
 * @brief          PID计算
 * @param[in]      PID结构体
 * @param[in]      测量值
 * @param[in]      期望值
 * @retval         返回空
 */
float PIDCalculate(PIDInstance *pid, float measure, float ref)
{
    // 堵转检测
    if (pid->Improve & PID_ErrorHandle)
        f_PID_ErrorHandle(pid);

  
    // 保存上次的测量值和误差,计算当前error
    pid->Measure = measure;
    pid->Ref = ref;
    pid->Err = pid->Ref - pid->Measure;

    // 如果在死区外,则计算PID
    if (abs(pid->Err) > pid->DeadBand)
    {
        // 基本的 PID 计算（位置式，无 DWT/dt）
        // 目标形式：Out = Kp*Error0 + Ki*ErrorInt + Kd*(Error0-Error1)
        pid->Pout = pid->Kp * pid->Err;

        // 积分“增量”（先按误差本身，后续可被梯形积分/变速积分修改）
        pid->ITerm = pid->Err;

        // 微分项（误差差分，Kd 在输出公式里统一相乘）
        pid->Dout = (pid->Err - pid->Last_Err);

        // 梯形积分
        if (pid->Improve & PID_Trapezoid_Intergral)
            f_Trapezoid_Intergral(pid);
        // 变速积分
        if (pid->Improve & PID_ChangingIntegrationRate)
            f_Changing_Integration_Rate(pid);
        // 微分先行
        if (pid->Improve & PID_Derivative_On_Measurement)
            f_Derivative_On_Measurement(pid);
        // 微分滤波器
        if (pid->Improve & PID_DerivativeFilter)
            f_Derivative_Filter(pid);
        // 积分限幅
        if (pid->Improve & PID_Integral_Limit)
            f_Integral_Limit(pid);

        // 误差积分（累加）：Ki==0 时清零，避免后续打开 Ki 时“历史积分爆炸”
        if (pid->Ki != 0.0f)
        {
            pid->Iout += pid->ITerm;
        }
        else
        {
            pid->Iout = 0.0f;
        }

        // 计算输出：Iout 是误差累计和，所以积分输出是 Ki * Iout
        pid->Output = pid->Pout + pid->Ki * pid->Iout + pid->Kd * pid->Dout;

        // 输出滤波
        if (pid->Improve & PID_OutputFilter)
            f_Output_Filter(pid);

        // 输出限幅
        f_Output_Limit(pid);
    }
    else // 进入死区, 则清空积分和输出
    {
        pid->Output = 0;
        pid->ITerm = 0;
        pid->Iout = 0;
    }

    // 保存当前数据,用于下次计算
    pid->Last_Measure = pid->Measure;
    pid->Last_Output = pid->Output;
    pid->Last_Dout = pid->Dout;
    pid->Last_Err = pid->Err;
    pid->Last_ITerm = pid->ITerm;

    return pid->Output;
}
