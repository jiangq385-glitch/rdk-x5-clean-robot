#include "servo.h"

#define ARM_COUNT               2u
#define ARM_JOINT_COUNT         6u
#define ARM_VALID_ALL           0x3Fu
#define ARM_BUS_DEFAULT_TIME_MS 800u
#define RAD_TO_DEG              57.2957795f

volatile arm_cmd_t arm_cmd[ARM_COUNT];

static const BusServoCal g_servo_cal[ARM_COUNT][ARM_JOINT_COUNT] = {
    {
        {1, 0.0f, 240.0f, 0, 1000, +1, 120.0f},
        {2, 0.0f, 240.0f, 0, 1000, +1, 120.0f},
        {3, 0.0f, 240.0f, 0, 1000, +1, 120.0f},
        {4, 0.0f, 240.0f, 0, 1000, +1, 120.0f},
        {5, 0.0f, 240.0f, 0, 1000, +1, 120.0f},
        {6, 0.0f, 240.0f, 0, 1000, +1, 120.0f},
    },
    {
        {7,  0.0f, 240.0f, 0, 1000, +1, 120.0f},
        {8,  0.0f, 240.0f, 0, 1000, +1, 120.0f},
        {9,  0.0f, 240.0f, 0, 1000, +1, 120.0f},
        {10, 0.0f, 240.0f, 0, 1000, +1, 120.0f},
        {11, 0.0f, 240.0f, 0, 1000, +1, 120.0f},
        {12, 0.0f, 240.0f, 0, 1000, +1, 120.0f},
    },
};

static const float arm0_home_rad[ARM_JOINT_COUNT] = {
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f
};

static const float arm1_home_rad[ARM_JOINT_COUNT] = {
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f
};

static const float *arm_home_rad(uint8_t arm_index)
{
    if (arm_index == 0u) return arm0_home_rad;
    if (arm_index == 1u) return arm1_home_rad;
    return NULL;
}

static float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static uint16_t deg_to_bus_pos(const BusServoCal *cal, float deg)
{
    float d = (float)cal->dir * deg + cal->offset_deg;
    float span_deg;
    float span_pos;
    float t;
    float pos_f;

    d = clampf(d, cal->min_deg, cal->max_deg);
    span_deg = cal->max_deg - cal->min_deg;
    span_pos = (float)(cal->max_pos - cal->min_pos);

    if (span_deg <= 1e-6f) return cal->min_pos;

    t = (d - cal->min_deg) / span_deg;
    pos_f = (float)cal->min_pos + t * span_pos;
    pos_f = clampf(pos_f, 0.0f, 1000.0f);

    return (uint16_t)(pos_f + 0.5f);
}

static void arm_bus_move_deg(uint8_t arm_index, const float deg[ARM_JOINT_COUNT], uint16_t time_ms)
{
    const BusServoCal *cal;
    uint16_t p[ARM_JOINT_COUNT];

    if (arm_index >= ARM_COUNT) return;
    if (time_ms == 0u) time_ms = ARM_BUS_DEFAULT_TIME_MS;

    cal = g_servo_cal[arm_index];
    for (uint8_t i = 0; i < ARM_JOINT_COUNT; i++) {
        p[i] = deg_to_bus_pos(&cal[i], deg[i]);
    }

    moveServos(6, time_ms,
               cal[0].id, p[0],
               cal[1].id, p[1],
               cal[2].id, p[2],
               cal[3].id, p[3],
               cal[4].id, p[4],
               cal[5].id, p[5]);
}

void arm_cmd_write_joints(uint8_t arm_index, uint8_t first_joint, float j0, float j1, float j2, uint16_t time_ms)
{
    volatile arm_cmd_t *cmd;

    if (arm_index >= ARM_COUNT) return;
    if (first_joint > 3u) return;

    cmd = &arm_cmd[arm_index];
    cmd->seq1++;
    cmd->joint_rad[first_joint] = j0;
    cmd->joint_rad[first_joint + 1u] = j1;
    cmd->joint_rad[first_joint + 2u] = j2;
    cmd->valid_mask |= (uint8_t)(0x07u << first_joint);
    cmd->update_mask |= (uint8_t)(0x07u << first_joint);
    cmd->time_ms = (time_ms == 0u) ? ARM_BUS_DEFAULT_TIME_MS : time_ms;
    if ((cmd->update_mask & ARM_VALID_ALL) == ARM_VALID_ALL) {
        cmd->pending = 1u;
    }
    cmd->seq2++;
}

void arm_cmd_write_home(uint8_t arm_index, uint16_t time_ms)
{
    const float *home = arm_home_rad(arm_index);
    volatile arm_cmd_t *cmd;

    if (home == NULL) return;

    cmd = &arm_cmd[arm_index];
    cmd->seq1++;
    for (uint8_t i = 0; i < ARM_JOINT_COUNT; i++) {
        cmd->joint_rad[i] = home[i];
    }
    cmd->valid_mask = ARM_VALID_ALL;
    cmd->update_mask = ARM_VALID_ALL;
    cmd->time_ms = (time_ms == 0u) ? ARM_BUS_DEFAULT_TIME_MS : time_ms;
    cmd->pending = 1u;
    cmd->seq2++;
}

static uint8_t arm_cmd_read(uint8_t arm_index, arm_cmd_t *snapshot)
{
    volatile arm_cmd_t *cmd;
    uint32_t seq1;
    uint32_t seq2;

    if (arm_index >= ARM_COUNT) return 0u;

    cmd = &arm_cmd[arm_index];
    do {
        seq1 = cmd->seq1;
        for (uint8_t i = 0; i < ARM_JOINT_COUNT; i++) {
            snapshot->joint_rad[i] = cmd->joint_rad[i];
        }
        snapshot->time_ms = cmd->time_ms;
        snapshot->valid_mask = cmd->valid_mask;
        snapshot->update_mask = cmd->update_mask;
        snapshot->pending = cmd->pending;
        snapshot->seq1 = cmd->seq1;
        snapshot->seq2 = cmd->seq2;
        seq2 = cmd->seq2;
    } while (seq1 != seq2);

    return snapshot->pending;
}

static void arm_cmd_clear_pending(uint8_t arm_index, uint32_t handled_seq)
{
    volatile arm_cmd_t *cmd;

    if (arm_index >= ARM_COUNT) return;

    cmd = &arm_cmd[arm_index];
    taskENTER_CRITICAL();
    if (cmd->seq1 != handled_seq || cmd->seq2 != handled_seq) {
        taskEXIT_CRITICAL();
        return;
    }

    cmd->seq1++;
    cmd->pending = 0u;
    cmd->update_mask = 0u;
    cmd->seq2++;
    taskEXIT_CRITICAL();
}

void arm_bus_task(void)
{
    for (uint8_t arm = 0; arm < ARM_COUNT; arm++) {
        arm_cmd_t snapshot;
        float deg[ARM_JOINT_COUNT];

        if (!arm_cmd_read(arm, &snapshot)) continue;

        if ((snapshot.valid_mask & ARM_VALID_ALL) != ARM_VALID_ALL) continue;
        for (uint8_t i = 0; i < ARM_JOINT_COUNT; i++) {
            deg[i] = snapshot.joint_rad[i] * RAD_TO_DEG;
        }

        arm_bus_move_deg(arm, deg, snapshot.time_ms);
        arm_cmd_clear_pending(arm, snapshot.seq2);
    }
}
