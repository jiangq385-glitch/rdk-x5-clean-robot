#include "message.h"
#include <limits.h>


//校验（和校验）
static uint8_t checksum(const uint8_t *data, uint8_t len)
{
    uint8_t sum = 0;
    for (uint8_t i = 0; i < len; i++) {
        sum += data[i];
    }
    return (uint8_t)(sum & 0xFF);

}



// 发送侧通用定点转换：
// 调用方负责决定输入量的物理单位，这里统一按“绝对值 × 1000 后取整”打包。
// 当前工程里用于速度、位姿、IMU 角度等浮点量发送，机械臂关节目标也遵循同一规则：
// 协议中传输的是 rad × 1000 后的定点整数，正负号由 flag 位表示。
static int16_t data_change(float v)
{
    float value = v*1000.0f; // 假设原始值是小数，放大1000倍转成整数
    if(value>=0) 
    {
        return (int16_t)(value + 0.5f); // 四舍五入
    } 
    else 
    {
        return (int16_t)(value - 0.5f); // 四舍五入
    }   
}

static uint8_t make_flag_from_signed(float vx, float vy, float vz)
{
    uint8_t flag = 0;
    if (vx >= 0) flag |= FLAG_X_POS;
    if (vy >= 0) flag |= FLAG_Y_POS;
    if (vz >= 0) flag |= FLAG_Z_POS;
    return flag;
}


void set_message(uint8_t type_id,float vx, float vy,float vz,uint8_t flag, uint8_t out[12])
{
    flag=make_flag_from_signed(vx,vy,vz);
    // 数据域只发送“绝对值”，符号由 flag 低三位表示。
    // 这套约定和机械臂关节协议一致：上位机发给 GD32 的原始值是 rad，
    // 发送前先做 abs(rad) * 1000，再把正负信息放进 flag。
    int16_t vx_int = data_change(fabsf(vx));
    int16_t vy_int = data_change(fabsf(vy));
    int16_t vz_int = data_change(fabsf(vz));

    out[0] = FRAME_HEADER;
    out[1] = type_id;
    out[2] = FRAME_LEN;

    out[3] = (uint8_t)((vx_int >> 8) & 0xFF); // vx高8位
    out[4] = (uint8_t)(vx_int & 0xFF);        // vx低8位
    out[5] = (uint8_t)((vy_int >> 8) & 0xFF);
    out[6] = (uint8_t)(vy_int & 0xFF);
    out[7] = (uint8_t)((vz_int >> 8) & 0xFF);
    out[8] = (uint8_t)(vz_int & 0xFF);
    out[9] = flag;
    out[10] = checksum(&out[1], 9); // 从ID开始算校验，长度为9
    out[11] = FRAME_TAIL;

}


 


//接收
//flag 判断
static float flag_judge(uint8_t flag,uint8_t flag_bit,float speed)
{
     return (flag & flag_bit) ? speed : -speed;

}

void speed_val(speed_frame_t *f)
{
// f->vx/vy/vz 里先放“绝对值”
    f->vx = flag_judge(f->flag, FLAG_X_POS, f->vx);
    f->vy = flag_judge(f->flag, FLAG_Y_POS, f->vy);
    f->vz = flag_judge(f->flag, FLAG_Z_POS, f->vz);
}

void host_frame_apply_sign(host_frame_t *f)
{
    f->x = flag_judge(f->flag, FLAG_X_POS, f->x);
    f->y = flag_judge(f->flag, FLAG_Y_POS, f->y);
    f->z = flag_judge(f->flag, FLAG_Z_POS, f->z);
}



static int16_t be_i16(uint8_t hi, uint8_t lo)
{
    return (int16_t)(((uint16_t)hi << 8) | lo);
}

bool feed_host_frame(uint8_t b, host_frame_t *out)
{
    static uint8_t buf[FRAME_SIZE];
    static uint8_t idx = 0;

    // 找帧头
    if (idx == 0)
    {
        if (b != FRAME_HEADER) return false;
        buf[idx++] = b;
        return false;
    }

    buf[idx++] = b;

    if (idx < FRAME_SIZE) return false;

    // 满 12 字节，开始判定
    idx = 0;

    if (buf[0] != FRAME_HEADER) return false;
    if (buf[11] != FRAME_TAIL)  return false;
    if (buf[2] != FRAME_LEN)    return false;

    if (checksum(&buf[1], 9) != buf[10]) return false;

    int16_t x_i = be_i16(buf[3], buf[4]);
    int16_t y_i = be_i16(buf[5], buf[6]);
    int16_t z_i = be_i16(buf[7], buf[8]);

    out->type_id = buf[1];
    out->x = (float)x_i / 1000.0f;
    out->y = (float)y_i / 1000.0f;
    out->z = (float)z_i / 1000.0f;
    out->flag = buf[9];

    // 协议约定：数据域为绝对值，符号由 flag 决定。
    // 先强制绝对值，避免对端误发负数导致 speed_val() 二次取负。
    out->x = fabsf(out->x);
    out->y = fabsf(out->y);
    out->z = fabsf(out->z);
    return true;
}

bool feed_speed_frame(uint8_t b, speed_frame_t *out)
{
    host_frame_t frame;

    if (!feed_host_frame(b, &frame)) return false;
    if (frame.type_id != TYPE_CMD_VEL) return false;

    out->vx = frame.x;
    out->vy = frame.y;
    out->vz = frame.z;
    out->flag = frame.flag;
    return true;
}

