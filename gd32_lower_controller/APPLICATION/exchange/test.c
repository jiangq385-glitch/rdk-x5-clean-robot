/**
 * gd32_ros2底盘驱动
 * - 读取左右编码器（增量式）
 * - 读取IMU（SPI/I2C接口）
 * - 发布 /odom 和 /imu
 * - 接收 /cmd_vel
 */

#include "main.h"
#include "usart.h"
#include "stdio.h"
#include "string.h"

// ============== 配置参数 ==============
#define WHEEL_BASE        0.3f        // 左右轮间距，单位：米
#define WHEEL_RADIUS      0.05f       // 轮子半径，单位：米
#define ENCODER_RESOLUTION 1024        // 编码器每圈脉冲数
#define GEAR_RATIO        30          // 减速比
#define ODOM_PUBLISH_HZ   50          // 里程计频率

// ============== 编码器数据 ==============
// 增量式编码器计数值（int16溢出自动回绕）
volatile int16_t left_encoder_count  = 0;
volatile int16_t right_encoder_count = 0;
static int16_t last_left_encoder     = 0;
static int16_t last_right_encoder    = 0;

// ============== 底盘状态 ==============
typedef struct {
    float x;           // 位置 x (米)
    float y;           // 位置 y (米)
    float theta;       // 航向角 (弧度)
    float v_left;      // 左轮速度 (m/s)
    float v_right;     // 右轮速度 (m/s)
    float v_linear;    // 底盘线速度 (m/s)
    float v_angular;   // 底盘角速度 (rad/s)
} ChassisState;

static ChassisState chassis = {0};

// ============== ROS2 消息缓冲区 ==============
#define TX_BUF_SIZE 512
static uint8_t tx_buf[TX_BUF_SIZE];

// ============== 定时相关 ==============
static uint32_t last_odom_time = 0;

// ============== 工具函数 ==============

// 将航向角转换为四元数
void yaw_to_quaternion(float yaw, float *qx, float *qy, float *qz, float *qw) {
    float half_yaw = yaw * 0.5f;
    *qx = 0.0f;
    *qy = 0.0f;
    *qz = sinf(half_yaw);
    *qw = cosf(half_yaw);
}

// 处理编码器溢出（int16回绕处理）
int32_t handle_encoder_overflow(int16_t new_val, int16_t last_val) {
    int32_t diff = (int32_t)(new_val - last_val);
    
    // 如果差值超过 ±32767，说明发生了溢出
    if (diff > 32767) {
        diff -= 65536;  // 负溢出
    } else if (diff < -32767) {
        diff += 65536;  // 正溢出
    }
    
    return diff;
}

// ============== 编码器读取（定时调用） ==============
void update_encoder(void) {
    // 读取当前编码器值（假设TIM2/TIM3捕获了编码器）
    int16_t cur_left  = (int16_t)TIM2->CNT;   // 左轮编码器
    int16_t cur_right = (int16_t)TIM3->CNT;   // 右轮编码器
    
    // 处理溢出，计算增量
    int32_t left_inc  = handle_encoder_overflow(cur_left,  last_left_encoder);
    int32_t right_inc = handle_encoder_overflow(cur_right, last_right_encoder);
    
    last_left_encoder  = cur_left;
    last_right_encoder = cur_right;
    
    // 将编码器增量转换为轮速 (m/s)
    // 每圈脉冲数 = ENCODER_RESOLUTION * GEAR_RATIO
    float pulses_per_rev = ENCODER_RESOLUTION * GEAR_RATIO;
    float dist_per_pulse = (2.0f * 3.14159f * WHEEL_RADIUS) / pulses_per_rev;
    
    // 需要知道采样周期，这里用固定频率
    float dt = 1.0f / ODOM_PUBLISH_HZ;
    
    chassis.v_left  = (left_inc  * dist_per_pulse) / dt;
    chassis.v_right = (right_inc * dist_per_pulse) / dt;
}

// ============== 里程计算（定时调用） ==============
void update_odometry(float dt) {
    // 差速底盘运动学
    chassis.v_linear  = (chassis.v_left + chassis.v_right) / 2.0f;
    chassis.v_angular = (chassis.v_right - chassis.v_left) / WHEEL_BASE;
    
    // 积分更新位姿
    chassis.theta += chassis.v_angular * dt;
    
    // 角度归一化到 [-π, π]
    while (chassis.theta > 3.14159f)  chassis.theta -= 2.0f * 3.14159f;
    while (chassis.theta < -3.14159f) chassis.theta += 2.0f * 3.14159f;
    
    chassis.x += chassis.v_linear * cosf(chassis.theta) * dt;
    chassis.y += chassis.v_linear * sinf(chassis.theta) * dt;
}

// ============== 发布 /odom 话题 ==============
// 使用 ros2 serial 协议自定义格式（也可换成micro-ROS的序列化方式）
void publish_odom(void) {
    float qx, qy, qz, qw;
    yaw_to_quaternion(chassis.theta, &qx, &qy, &qz, &qw);
    
    // 组装自定义协议（换你自己定义的格式）
    // 格式: HEADER(0xAA 0x55) | TYPE | LEN | DATA | CRC16
    uint8_t type = 0x01;  // 0x01 = odom
    
    // 使用 sprintf 组包（实际项目建议用更高效的二进制格式）
    int len = snprintf((char*)tx_buf, TX_BUF_SIZE,
        "{\"type\":\"odom\","
        "\"x\":%.4f,\"y\":%.4f,\"theta\":%.4f,"
        "\"vx\":%.4f,\"vy\":%.4f,\"omega\":%.4f,"
        "\"qx\":%.4f,\"qy\":%.4f,\"qz\":%.4f,\"qw\":%.4f}\n",
        chassis.x, chassis.y, chassis.theta,
        chassis.v_linear, 0.0f, chassis.v_angular,
        qx, qy, qz, qw
    );
    
    HAL_UART_Transmit(&huart2, tx_buf, len, 100);
}

// ============== 发布 /imu 话题 ==============
void publish_imu(float gx, float gy, float gz, 
                 float ax, float ay, float az) {
    uint8_t type = 0x02;  // 0x02 = imu
    
    int len = snprintf((char*)tx_buf, TX_BUF_SIZE,
        "{\"type\":\"imu\","
        "\"gx\":%.4f,\"gy\":%.4f,\"gz\":%.4f,"
        "\"ax\":%.4f,\"ay\":%.4f,\"az\":%.4f}\n",
        gx, gy, gz, ax, ay, az
    );
    
    HAL_UART_Transmit(&huart2, tx_buf, len, 100);
}

// ============== 读取IMU（假设用SPI/I2C的MPU6050） ==============
void read_imu_data(float *gx, float *gy, float *gz,
                   float *ax, float *ay, float *az) {
    // 读取陀螺仪原始值（需要你自己实现I2C/SPI读取）
    // 这里用占位，实际需要根据你的IMU型号写驱动
    uint8_t reg_gyro_xh = 0x43;
    uint8_t data[6];
    
    // HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDR, &reg_gyro_xh, 1, 100);
    // HAL_I2C_Master_Receive(&hi2c1, MPU6050_ADDR, data, 6, 100);
    
    // 转换为实际物理量（需要标定和转换系数）
    // *gx = ((int16_t)(data[0]<<8 | data[1])) / 32768.0f * 500.0f;  // dps
    // ...
}

// ============== 接收 /cmd_vel 回调 ==============
// 来自ROS2的串口数据解析
void parse_uart_rx(uint8_t byte) {
    static uint8_t rx_buf[64];
    static uint8_t rx_len = 0;
    
    if (byte == '\n') {
        rx_buf[rx_len] = 0;
        rx_len = 0;
        
        // 简单解析 cmd_vel JSON
        if (strstr((char*)rx_buf, "\"type\":\"cmd\"")) {
            float vx, omega;
            sscanf((char*)rx_buf, "%*[^\"]\"vx\":%f,\"omega\":%f", &vx, &omega);
            
            // 差速底盘逆运动学：转换为左右轮目标速度
            float target_v_left  = vx - omega * WHEEL_BASE / 2.0f;
            float target_v_right = vx + omega * WHEEL_BASE / 2.0f;
            
            // TODO: 将目标速度传给PID控制器输出PWM
            set_motor_speed(MOTOR_LEFT,  target_v_left);
            set_motor_speed(MOTOR_RIGHT, target_v_right);
        }
    } else {
        if (rx_len < 63) {
            rx_buf[rx_len++] = byte;
        }
    }
}

// ============== 主循环（1ms systick） ==============
void chassis_control_loop(void) {
    uint32_t now = HAL_GetTick();
    uint32_t dt_ms = now - last_odom_time;
    
    if (dt_ms >= (1000 / ODOM_PUBLISH_HZ)) {
        float dt = dt_ms / 1000.0f;
        last_odom_time = now;
        
        // 1. 更新编码器
        update_encoder();
        
        // 2. 更新里程
        update_odometry(dt);
        
        // 3. 发布里程计
        publish_odom();
        
        // 4. 读取并发布IMU（IMU频率通常更高，这里复用）
        float gx, gy, gz, ax, ay, az;
        read_imu_data(&gx, &gy, &gz, &ax, &ay, &az);
        publish_imu(gx, gy, gz, ax, ay, az);
    }
}

// ============== 串口接收中断 ==============
void USART2_IRQHandler(void) {
    if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE)) {
        uint8_t byte = (uint8_t)(USART2->RDR);
        parse_uart_rx(byte);
    }
}