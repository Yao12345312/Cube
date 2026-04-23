#pragma once

#include "cmsis_os2.h"

typedef struct
{
    float roll;
    float pitch;
    float yaw;
} Attitude_t;

// 全局共享变量
extern Attitude_t g_attitude;

extern osMutexId_t g_att_mutex;

