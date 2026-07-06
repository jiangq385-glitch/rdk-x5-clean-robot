/**
 ******************************************************************************
 * @file    kalman filter.c
 * @author  Wang Hongxi
 * @version V1.2.2
 * @date    2022/1/8
 * @brief   C implementation of kalman filter
 ******************************************************************************
 * @attention
 * 该卡尔曼滤波器可以在传感器采样频率不同的情况下，动态调整矩阵H R和K的维数与数值。
 * This implementation of kalman filter can dynamically adjust dimension and
 * value of matrix H R and K according to the measurement validity under any
 * circumstance that the sampling rate of component sensors are different.
 *
 * 因此矩阵H和R的初始化会与矩阵P A和Q有所不同。另外的，在初始化量测向量z时需要额外写
 * 入传感器量测所对应的状态与这个量测的方式，详情请见例程
 * Therefore, the initialization of matrix P, F, and Q is sometimes different
 * from that of matrices H R. when initialization. Additionally, the corresponding
 * state and the method of the measurement should be provided when initializing
 * measurement vector z. For more details, please see the example.
 *
 * 若不需要动态调整量测向量z，可简单将结构体中的Use_Auto_Adjustment初始化为0，并像初
 * 始化矩阵P那样用常规方式初始化z H R即可。
 * If automatic adjustment is not required, assign zero to the UseAutoAdjustment
 * and initialize z H R in the normal way as matrix P.
 *
 * 要求量测向量z与控制向量u在传感器回调函数中更新。整数0意味着量测无效，即自上次卡尔曼
 * 滤波更新后无传感器数据更新。因此量测向量z与控制向量u会在卡尔曼滤波更新过程中被清零
 * MeasuredVector and ControlVector are required to be updated in the sensor
 * callback function. Integer 0 in measurement vector z indicates the invalidity
 * of current measurement, so MeasuredVector and ControlVector will be reset
 * (to 0) during each update.
 *
 * 此外，矩阵P过度收敛后滤波器将难以再适应状态的缓慢变化，从而产生滤波估计偏差。该算法
 * 通过限制矩阵P最小值的方法，可有效抑制滤波器的过度收敛，详情请见例程。
 * Additionally, the excessive convergence of matrix P will make filter incapable
 * of adopting the slowly changing state. This implementation can effectively
 * suppress filter excessive convergence through boundary limiting for matrix P.
 * For more details, please see the example.
 *
 * @example:
 * x =
 *   |   height   |
 *   |  velocity  |
 *   |acceleration|
 *
 * KalmanFilter_t Height_KF;
 *
 * void INS_Task_Init(void)
 * {
 *     static float P_Init[9] =
 *     {
 *         10, 0, 0,
 *         0, 30, 0,
 *         0, 0, 10,
 *     };
 *     static float F_Init[9] =
 *     {
 *         1, dt, 0.5*dt*dt,
 *         0, 1, dt,
 *         0, 0, 1,
 *     };
 *     static float Q_Init[9] =
 *     {
 *         0.25*dt*dt*dt*dt, 0.5*dt*dt*dt, 0.5*dt*dt,
 *         0.5*dt*dt*dt,        dt*dt,         dt,
 *         0.5*dt*dt,              dt,         1,
 *     };
 *
 *     // 设置最小方差
 *     static float state_min_variance[3] = {0.03, 0.005, 0.1};
 *
 *     // 开启自动调整
 *     Height_KF.UseAutoAdjustment = 1;
 *
 *     // 气压测得高度 GPS测得高度 加速度计测得z轴运动加速度
 *     static uint8_t measurement_reference[3] = {1, 1, 3}
 *
 *     static float measurement_degree[3] = {1, 1, 1}
 *     // 根据measurement_reference与measurement_degree生成H矩阵如下（在当前周期全部测量数据有效情况下）
 *       |1   0   0|
 *       |1   0   0|
 *       |0   0   1|
 *
 *     static float mat_R_diagonal_elements = {30, 25, 35}
 *     //根据mat_R_diagonal_elements生成R矩阵如下（在当前周期全部测量数据有效情况下）
 *       |30   0   0|
 *       | 0  25   0|
 *       | 0   0  35|
 *
 *     Kalman_Filter_Init(&Height_KF, 3, 0, 3);
 *
 *     // 设置矩阵值
 *     memcpy(Height_KF.P_data, P_Init, sizeof(P_Init));
 *     memcpy(Height_KF.F_data, F_Init, sizeof(F_Init));
 *     memcpy(Height_KF.Q_data, Q_Init, sizeof(Q_Init));
 *     memcpy(Height_KF.MeasurementMap, measurement_reference, sizeof(measurement_reference));
 *     memcpy(Height_KF.MeasurementDegree, measurement_degree, sizeof(measurement_degree));
 *     memcpy(Height_KF.MatR_DiagonalElements, mat_R_diagonal_elements, sizeof(mat_R_diagonal_elements));
 *     memcpy(Height_KF.StateMinVariance, state_min_variance, sizeof(state_min_variance));
 * }
 *
 * void INS_Task(void const *pvParameters)
 * {
 *     // 循环更新
 *     Kalman_Filter_Update(&Height_KF);
 *     vTaskDelay(ts);
 * }
 *
 * // 测量数据更新应按照以下形式 即向MeasuredVector赋值
 * void Barometer_Read_Over(void)
 * {
 *     ......
 *     INS_KF.MeasuredVector[0] = baro_height;
 * }
 * void GPS_Read_Over(void)
 * {
 *     ......
 *     INS_KF.MeasuredVector[1] = GPS_height;
 * }
 * void Acc_Data_Process(void)
 * {
 *     ......
 *     INS_KF.MeasuredVector[2] = acc.z;
 * }
 ******************************************************************************
 */

#include "kalman_filter.h"
#include <string.h>

/*
 * 本文件实现的是标准线性卡尔曼滤波（Kalman Filter, KF），并支持“量测向量动态有效性”。
 *
 * 记号约定（与结构体字段对应）：
 * - xhat      : \hat{x}(k|k)   后验状态估计（融合量测后）
 * - xhatminus : \hat{x}(k|k-1) 先验状态估计（仅预测）
 * - P         : P(k|k)         后验协方差
 * - Pminus    : P(k|k-1)       先验协方差
 * - F         : 状态转移矩阵（也常写作 A）
 * - B, u      : 控制矩阵/控制输入（本工程视觉误差滤波一般 uSize=0，不使用）
 * - H         : 观测矩阵
 * - z         : 观测向量（由 MeasuredVector 复制/重构而来）
 * - Q         : 过程噪声协方差
 * - R         : 量测噪声协方差
 * - K         : 卡尔曼增益
 *
 * 标准 KF 五式（本文件函数一一对应）：
 * 1) x^- = F x + B u                      -> Kalman_Filter_xhatMinusUpdate
 * 2) P^- = F P F^T + Q                    -> Kalman_Filter_PminusUpdate
 * 3) K   = P^- H^T (H P^- H^T + R)^{-1}   -> Kalman_Filter_SetK
 * 4) x   = x^- + K (z - H x^-)            -> Kalman_Filter_xhatUpdate
 * 5) P   = P^- - K H P^-                  -> Kalman_Filter_P_Update
 *
 * 动态有效量测机制（UseAutoAdjustment=1）：
 * - 本实现将 MeasuredVector 中“数值为 0”视为该维量测无效
 * - H_K_R_Adjustment 会根据有效量测数量重构 z/H/R，并动态改写矩阵维度（numRows/numCols）
 * - 若 MeasurementValidNum==0，则 Kalman_Filter_Update 走“仅预测”分支
 *
 * 注意：当你的真实量测可能合法地等于 0（例如误差刚好为 0）时，该机制会把它当成“无量测”。
 */

uint16_t sizeof_float, sizeof_double;

static void H_K_R_Adjustment(KalmanFilter_t *kf);

/**
 * @brief 初始化矩阵维度信息并为矩阵分配空间
 *
 * @param kf kf类型定义
 * @param xhatSize 状态变量维度
 * @param uSize 控制变量维度
 * @param zSize 观测量维度
 */
void Kalman_Filter_Init(KalmanFilter_t *kf, uint8_t xhatSize, uint8_t uSize, uint8_t zSize)
{
    sizeof_float = sizeof(float);
    sizeof_double = sizeof(double);

    kf->xhatSize = xhatSize;
    kf->uSize = uSize;
    kf->zSize = zSize;

    kf->MeasurementValidNum = 0;

    // --- 与“动态有效量测”相关的映射/参数 ---
    // MeasurementMap[i]      : 第 i 个量测对应状态向量的第几维（从 1 开始计数）
    // MeasurementDegree[i]   : 该量测对应的 H 矩阵元素（例如直接观测就是 1）
    // MatR_DiagonalElements  : 每个量测的方差（用于构造 R 的对角线）
    // StateMinVariance       : P 的对角线下界，防止过度收敛
    kf->MeasurementMap = (uint8_t *)user_malloc(sizeof(uint8_t) * zSize);
    memset(kf->MeasurementMap, 0, sizeof(uint8_t) * zSize);
    kf->MeasurementDegree = (float *)user_malloc(sizeof_float * zSize);
    memset(kf->MeasurementDegree, 0, sizeof_float * zSize);
    kf->MatR_DiagonalElements = (float *)user_malloc(sizeof_float * zSize);
    memset(kf->MatR_DiagonalElements, 0, sizeof_float * zSize);
    kf->StateMinVariance = (float *)user_malloc(sizeof_float * xhatSize);
    memset(kf->StateMinVariance, 0, sizeof_float * xhatSize);
    kf->temp = (uint8_t *)user_malloc(sizeof(uint8_t) * zSize);
    memset(kf->temp, 0, sizeof(uint8_t) * zSize);

    // --- 供用户读写的向量缓存 ---
    // FilteredValue  : 每次 Update() 后会把 xhat 拷贝到这里，作为返回指针
    // MeasuredVector : 用户在“传感器回调/数据更新处”写入（Update 内部会复制到 z 并清零）
    // ControlVector  : 用户写入控制量（若 uSize=0 可忽略）
    kf->FilteredValue = (float *)user_malloc(sizeof_float * xhatSize);
    memset(kf->FilteredValue, 0, sizeof_float * xhatSize);
    kf->MeasuredVector = (float *)user_malloc(sizeof_float * zSize);
    memset(kf->MeasuredVector, 0, sizeof_float * zSize);
    kf->ControlVector = (float *)user_malloc(sizeof_float * uSize);
    memset(kf->ControlVector, 0, sizeof_float * uSize);

    // xhat : \hat{x}(k|k) 后验估计（融合量测之后）
    kf->xhat_data = (float *)user_malloc(sizeof_float * xhatSize);
    memset(kf->xhat_data, 0, sizeof_float * xhatSize);
    Matrix_Init(&kf->xhat, kf->xhatSize, 1, (float *)kf->xhat_data);

    // xhatminus : \hat{x}(k|k-1) 先验估计（仅预测）
    kf->xhatminus_data = (float *)user_malloc(sizeof_float * xhatSize);
    memset(kf->xhatminus_data, 0, sizeof_float * xhatSize);
    Matrix_Init(&kf->xhatminus, kf->xhatSize, 1, (float *)kf->xhatminus_data);

    if (uSize != 0)
    {
            // control vector u（uSize>0 才会启用）
        kf->u_data = (float *)user_malloc(sizeof_float * uSize);
        memset(kf->u_data, 0, sizeof_float * uSize);
        Matrix_Init(&kf->u, kf->uSize, 1, (float *)kf->u_data);
    }

    // measurement vector z（内部工作向量；由 MeasuredVector 复制/重构而来）
    kf->z_data = (float *)user_malloc(sizeof_float * zSize);
    memset(kf->z_data, 0, sizeof_float * zSize);
    Matrix_Init(&kf->z, kf->zSize, 1, (float *)kf->z_data);

    // covariance matrix P(k|k)：后验协方差
    kf->P_data = (float *)user_malloc(sizeof_float * xhatSize * xhatSize);
    memset(kf->P_data, 0, sizeof_float * xhatSize * xhatSize);
    Matrix_Init(&kf->P, kf->xhatSize, kf->xhatSize, (float *)kf->P_data);

    // covariance matrix P(k|k-1)：先验协方差
    kf->Pminus_data = (float *)user_malloc(sizeof_float * xhatSize * xhatSize);
    memset(kf->Pminus_data, 0, sizeof_float * xhatSize * xhatSize);
    Matrix_Init(&kf->Pminus, kf->xhatSize, kf->xhatSize, (float *)kf->Pminus_data);

    // state transition matrix F 与其转置 FT
    kf->F_data = (float *)user_malloc(sizeof_float * xhatSize * xhatSize);
    kf->FT_data = (float *)user_malloc(sizeof_float * xhatSize * xhatSize);
    memset(kf->F_data, 0, sizeof_float * xhatSize * xhatSize);
    memset(kf->FT_data, 0, sizeof_float * xhatSize * xhatSize);
    Matrix_Init(&kf->F, kf->xhatSize, kf->xhatSize, (float *)kf->F_data);
    Matrix_Init(&kf->FT, kf->xhatSize, kf->xhatSize, (float *)kf->FT_data);

    if (uSize != 0)
    {
            // control matrix B（uSize>0 才会启用）
        kf->B_data = (float *)user_malloc(sizeof_float * xhatSize * uSize);
        memset(kf->B_data, 0, sizeof_float * xhatSize * uSize);
        Matrix_Init(&kf->B, kf->xhatSize, kf->uSize, (float *)kf->B_data);
    }

        // measurement matrix H 与其转置 HT
    kf->H_data = (float *)user_malloc(sizeof_float * zSize * xhatSize);
    kf->HT_data = (float *)user_malloc(sizeof_float * xhatSize * zSize);
    memset(kf->H_data, 0, sizeof_float * zSize * xhatSize);
    memset(kf->HT_data, 0, sizeof_float * xhatSize * zSize);
    Matrix_Init(&kf->H, kf->zSize, kf->xhatSize, (float *)kf->H_data);
    Matrix_Init(&kf->HT, kf->xhatSize, kf->zSize, (float *)kf->HT_data);

    // process noise covariance matrix Q（过程噪声）
    kf->Q_data = (float *)user_malloc(sizeof_float * xhatSize * xhatSize);
    memset(kf->Q_data, 0, sizeof_float * xhatSize * xhatSize);
    Matrix_Init(&kf->Q, kf->xhatSize, kf->xhatSize, (float *)kf->Q_data);

    // measurement noise covariance matrix R（量测噪声）
    kf->R_data = (float *)user_malloc(sizeof_float * zSize * zSize);
    memset(kf->R_data, 0, sizeof_float * zSize * zSize);
    Matrix_Init(&kf->R, kf->zSize, kf->zSize, (float *)kf->R_data);

    // kalman gain K（卡尔曼增益）
    kf->K_data = (float *)user_malloc(sizeof_float * xhatSize * zSize);
    memset(kf->K_data, 0, sizeof_float * xhatSize * zSize);
    Matrix_Init(&kf->K, kf->xhatSize, kf->zSize, (float *)kf->K_data);

    kf->S_data = (float *)user_malloc(sizeof_float * kf->xhatSize * kf->xhatSize);
    kf->temp_matrix_data = (float *)user_malloc(sizeof_float * kf->xhatSize * kf->xhatSize);
    kf->temp_matrix_data1 = (float *)user_malloc(sizeof_float * kf->xhatSize * kf->xhatSize);
    kf->temp_vector_data = (float *)user_malloc(sizeof_float * kf->xhatSize);
    kf->temp_vector_data1 = (float *)user_malloc(sizeof_float * kf->xhatSize);
    Matrix_Init(&kf->S, kf->xhatSize, kf->xhatSize, (float *)kf->S_data);
    Matrix_Init(&kf->temp_matrix, kf->xhatSize, kf->xhatSize, (float *)kf->temp_matrix_data);
    Matrix_Init(&kf->temp_matrix1, kf->xhatSize, kf->xhatSize, (float *)kf->temp_matrix_data1);
    Matrix_Init(&kf->temp_vector, kf->xhatSize, 1, (float *)kf->temp_vector_data);
    Matrix_Init(&kf->temp_vector1, kf->xhatSize, 1, (float *)kf->temp_vector_data1);

    kf->SkipEq1 = 0;
    kf->SkipEq2 = 0;
    kf->SkipEq3 = 0;
    kf->SkipEq4 = 0;
    kf->SkipEq5 = 0;
}

void Kalman_Filter_Measure(KalmanFilter_t *kf)
{
    // 矩阵H K R根据量测情况自动调整
    // matrix H K R auto adjustment
    if (kf->UseAutoAdjustment != 0)
        H_K_R_Adjustment(kf);
    else
    {
        // 不使用自动调整：直接把 MeasuredVector 作为 z，维度固定为 zSize
        memcpy(kf->z_data, kf->MeasuredVector, sizeof_float * kf->zSize);
        memset(kf->MeasuredVector, 0, sizeof_float * kf->zSize);
    }

    // uSize==0 时 u_data 未初始化/未分配，这里显式保护，避免潜在的未定义行为。
    if (kf->uSize > 0)
    {
        memcpy(kf->u_data, kf->ControlVector, sizeof_float * kf->uSize);
    }
}

void Kalman_Filter_xhatMinusUpdate(KalmanFilter_t *kf)
{
    if (!kf->SkipEq1)
    {
        if (kf->uSize > 0)
        {
            // 公式(1)：x^- = F x + B u
            kf->temp_vector.numRows = kf->xhatSize;
            kf->temp_vector.numCols = 1;
            kf->MatStatus = Matrix_Multiply(&kf->F, &kf->xhat, &kf->temp_vector);
            kf->temp_vector1.numRows = kf->xhatSize;
            kf->temp_vector1.numCols = 1;
            kf->MatStatus = Matrix_Multiply(&kf->B, &kf->u, &kf->temp_vector1);
            kf->MatStatus = Matrix_Add(&kf->temp_vector, &kf->temp_vector1, &kf->xhatminus);
        }
        else
        {
            // 公式(1)（无控制输入）：x^- = F x
            kf->MatStatus = Matrix_Multiply(&kf->F, &kf->xhat, &kf->xhatminus);
        }
    }
}

void Kalman_Filter_PminusUpdate(KalmanFilter_t *kf)
{
    if (!kf->SkipEq2)
    {
        // 公式(2)：P^- = F P F^T + Q
        kf->MatStatus = Matrix_Transpose(&kf->F, &kf->FT);
        kf->MatStatus = Matrix_Multiply(&kf->F, &kf->P, &kf->Pminus);
        kf->temp_matrix.numRows = kf->Pminus.numRows;
        kf->temp_matrix.numCols = kf->FT.numCols;
        kf->MatStatus = Matrix_Multiply(&kf->Pminus, &kf->FT, &kf->temp_matrix); // temp_matrix = F P(k-1) FT
        kf->MatStatus = Matrix_Add(&kf->temp_matrix, &kf->Q, &kf->Pminus);
    }
}
void Kalman_Filter_SetK(KalmanFilter_t *kf)
{
    if (!kf->SkipEq3)
    {
        // 公式(3)：K = P^- H^T (H P^- H^T + R)^{-1}
        kf->MatStatus = Matrix_Transpose(&kf->H, &kf->HT); // z|x => x|z
        kf->temp_matrix.numRows = kf->H.numRows;
        kf->temp_matrix.numCols = kf->Pminus.numCols;
        kf->MatStatus = Matrix_Multiply(&kf->H, &kf->Pminus, &kf->temp_matrix); // temp_matrix = H·P'(k)
        kf->temp_matrix1.numRows = kf->temp_matrix.numRows;
        kf->temp_matrix1.numCols = kf->HT.numCols;
        kf->MatStatus = Matrix_Multiply(&kf->temp_matrix, &kf->HT, &kf->temp_matrix1); // temp_matrix1 = H·P'(k)·HT
        kf->S.numRows = kf->R.numRows;
        kf->S.numCols = kf->R.numCols;
        kf->MatStatus = Matrix_Add(&kf->temp_matrix1, &kf->R, &kf->S); // S = H P'(k) HT + R
        kf->MatStatus = Matrix_Inverse(&kf->S, &kf->temp_matrix1);     // temp_matrix1 = inv(H·P'(k)·HT + R)
        kf->temp_matrix.numRows = kf->Pminus.numRows;
        kf->temp_matrix.numCols = kf->HT.numCols;
        kf->MatStatus = Matrix_Multiply(&kf->Pminus, &kf->HT, &kf->temp_matrix); // temp_matrix = P'(k)·HT
        kf->MatStatus = Matrix_Multiply(&kf->temp_matrix, &kf->temp_matrix1, &kf->K);
    }
}
void Kalman_Filter_xhatUpdate(KalmanFilter_t *kf)
{
    if (!kf->SkipEq4)
    {
        // 公式(4)：x = x^- + K (z - H x^-)
        kf->temp_vector.numRows = kf->H.numRows;
        kf->temp_vector.numCols = 1;
        kf->MatStatus = Matrix_Multiply(&kf->H, &kf->xhatminus, &kf->temp_vector); // temp_vector = H xhat'(k)
        kf->temp_vector1.numRows = kf->z.numRows;
        kf->temp_vector1.numCols = 1;
        kf->MatStatus = Matrix_Subtract(&kf->z, &kf->temp_vector, &kf->temp_vector1); // temp_vector1 = z(k) - H·xhat'(k)
        kf->temp_vector.numRows = kf->K.numRows;
        kf->temp_vector.numCols = 1;
        kf->MatStatus = Matrix_Multiply(&kf->K, &kf->temp_vector1, &kf->temp_vector); // temp_vector = K(k)·(z(k) - H·xhat'(k))
        kf->MatStatus = Matrix_Add(&kf->xhatminus, &kf->temp_vector, &kf->xhat);
    }
}
void Kalman_Filter_P_Update(KalmanFilter_t *kf)
{
    if (!kf->SkipEq5)
    {
        // 公式(5)：P = P^- - K H P^-
        kf->temp_matrix.numRows = kf->K.numRows;
        kf->temp_matrix.numCols = kf->H.numCols;
        kf->temp_matrix1.numRows = kf->temp_matrix.numRows;
        kf->temp_matrix1.numCols = kf->Pminus.numCols;
        kf->MatStatus = Matrix_Multiply(&kf->K, &kf->H, &kf->temp_matrix);                 // temp_matrix = K(k)·H
        kf->MatStatus = Matrix_Multiply(&kf->temp_matrix, &kf->Pminus, &kf->temp_matrix1); // temp_matrix1 = K(k)·H·P'(k)
        kf->MatStatus = Matrix_Subtract(&kf->Pminus, &kf->temp_matrix1, &kf->P);
    }
}

/**
 * @brief 执行卡尔曼滤波黄金五式,提供了用户定义函数,可以替代五个中的任意一个环节,方便自行扩展为EKF/UKF/ESKF/AUKF等
 * 
 * @param kf kf类型定义
 * @return float* 返回滤波值
 */
float *Kalman_Filter_Update(KalmanFilter_t *kf)
{
    // 0. 获取量测信息
    Kalman_Filter_Measure(kf);
    if (kf->User_Func0_f != NULL)
        kf->User_Func0_f(kf);

    // 先验估计
    // 1. xhat'(k)= A·xhat(k-1) + B·u
    Kalman_Filter_xhatMinusUpdate(kf);
    if (kf->User_Func1_f != NULL)
        kf->User_Func1_f(kf);

    // 预测更新
    // 2. P'(k) = A·P(k-1)·AT + Q
    Kalman_Filter_PminusUpdate(kf);
    if (kf->User_Func2_f != NULL)
        kf->User_Func2_f(kf);

    if (kf->MeasurementValidNum != 0 || kf->UseAutoAdjustment == 0)
    {
        // 量测更新
        // 3. K(k) = P'(k)·HT / (H·P'(k)·HT + R)
        Kalman_Filter_SetK(kf);

        if (kf->User_Func3_f != NULL)
            kf->User_Func3_f(kf);

        // 融合
        // 4. xhat(k) = xhat'(k) + K(k)·(z(k) - H·xhat'(k))
        Kalman_Filter_xhatUpdate(kf);

        if (kf->User_Func4_f != NULL)
            kf->User_Func4_f(kf);

        // 修正方差
        // 5. P(k) = (1-K(k)·H)·P'(k) ==> P(k) = P'(k)-K(k)·H·P'(k)
        Kalman_Filter_P_Update(kf);
    }
    else
    {
        // 无有效量测,仅预测
        // xhat(k) = xhat'(k)
        // P(k) = P'(k)
        memcpy(kf->xhat_data, kf->xhatminus_data, sizeof_float * kf->xhatSize);
        memcpy(kf->P_data, kf->Pminus_data, sizeof_float * kf->xhatSize * kf->xhatSize);
    }

    // 自定义函数,可以提供后处理等
    if (kf->User_Func5_f != NULL)
        kf->User_Func5_f(kf);

    // 避免滤波器过度收敛
    // suppress filter excessive convergence
    for (uint8_t i = 0; i < kf->xhatSize; ++i)
    {
        // 仅对 P 的对角线做下界限制：保证每个状态至少保留一定不确定性
        if (kf->P_data[i * kf->xhatSize + i] < kf->StateMinVariance[i])
            kf->P_data[i * kf->xhatSize + i] = kf->StateMinVariance[i];
    }

    memcpy(kf->FilteredValue, kf->xhat_data, sizeof_float * kf->xhatSize);

    if (kf->User_Func6_f != NULL)
        kf->User_Func6_f(kf);

    return kf->FilteredValue;
}

static void H_K_R_Adjustment(KalmanFilter_t *kf)
{
    kf->MeasurementValidNum = 0;

    memcpy(kf->z_data, kf->MeasuredVector, sizeof_float * kf->zSize);
    memset(kf->MeasuredVector, 0, sizeof_float * kf->zSize);

    // 识别量测数据有效性并调整矩阵 H/R/K 的维数与内容
    // recognize measurement validity and adjust matrices H R K
    memset(kf->R_data, 0, sizeof_float * kf->zSize * kf->zSize);
    memset(kf->H_data, 0, sizeof_float * kf->xhatSize * kf->zSize);
    for (uint8_t i = 0; i < kf->zSize; ++i)
    {
        // 约定：z_data[i] == 0 代表“本周期该维量测无效（未更新）”。
        // ⚠ 若你的真实测量值可能合法地等于 0（例如误差刚好为 0），该策略会误判为无效量测。
        if (kf->z_data[i] != 0)
        {
            // 重构向量z
            // rebuild vector z
            kf->z_data[kf->MeasurementValidNum] = kf->z_data[i];
            kf->temp[kf->MeasurementValidNum] = i;
            // 重构矩阵H
            // rebuild matrix H
            // MeasurementMap 从 1 开始计数，因此要 -1
            kf->H_data[kf->xhatSize * kf->MeasurementValidNum + kf->MeasurementMap[i] - 1] = kf->MeasurementDegree[i];
            kf->MeasurementValidNum++;
        }
    }
    for (uint8_t i = 0; i < kf->MeasurementValidNum; ++i)
    {
        // 重构矩阵R
        // rebuild matrix R
        kf->R_data[i * kf->MeasurementValidNum + i] = kf->MatR_DiagonalElements[kf->temp[i]];
    }

    // 调整矩阵维数
    // adjust the dimensions of system matrices
    kf->H.numRows = kf->MeasurementValidNum;
    kf->H.numCols = kf->xhatSize;
    kf->HT.numRows = kf->xhatSize;
    kf->HT.numCols = kf->MeasurementValidNum;
    kf->R.numRows = kf->MeasurementValidNum;
    kf->R.numCols = kf->MeasurementValidNum;
    kf->K.numRows = kf->xhatSize;
    kf->K.numCols = kf->MeasurementValidNum;
    kf->z.numRows = kf->MeasurementValidNum;
}
