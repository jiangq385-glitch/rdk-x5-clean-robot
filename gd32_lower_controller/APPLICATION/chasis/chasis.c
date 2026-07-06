#include "chasis.h"
extern vel_cmd_t vel_cmd;
void chasis_task(void)
{
    vel_cmd.LEFT_ADDR=2;
    vel_cmd.RIGHT_ADDR=1;
    Differential_kinematics(&vel_cmd);
    // 6) 发送到电机（你的速度模式接口）
    Emm_V5_Vel_Control(vel_cmd.LEFT_ADDR,  vel_cmd.dir_l, vel_cmd.rpm_l, 0, false);
    Emm_V5_Vel_Control(vel_cmd.RIGHT_ADDR, vel_cmd.dir_r, vel_cmd.rpm_r, 0, false);

}
