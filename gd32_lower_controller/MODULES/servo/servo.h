#ifndef __SERVO_H
#define __SERVO_H

#include "main.h"

typedef struct {
    uint8_t  id;
    float    min_deg;
    float    max_deg;
    uint16_t min_pos;
    uint16_t max_pos;
    int8_t   dir;
    float    offset_deg;
} BusServoCal;

typedef struct {
    float joint_rad[6];
    uint16_t time_ms;
    uint8_t valid_mask;
    uint8_t update_mask;
    uint8_t pending;
    uint32_t seq1;
    uint32_t seq2;
} arm_cmd_t;

extern volatile arm_cmd_t arm_cmd[2];

void arm_cmd_write_joints(uint8_t arm_index, uint8_t first_joint,
                          float j0, float j1, float j2,
                          uint16_t time_ms);
void arm_cmd_write_home(uint8_t arm_index, uint16_t time_ms);
void arm_bus_task(void);

#endif
