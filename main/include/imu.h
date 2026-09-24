#ifndef IMU_STRUCT_H
#define IMU_STRUCT_H

#include <inttypes.h>
#include "bno085.h"

typedef enum __attribute__((packed)) {
    BNO_TYPE_EMPTY = 0,
    BNO_TYPE_LINEAR_ACCEL = 1,
    BNO_TYPE_GAME_ROTATION = 2
} BNO_DataType_t;

typedef struct __attribute__((packed)) {
    float x;
    float y;
    float z;
} LinearAccel_t; // 12 bytes

typedef struct __attribute__((packed)) {
    float i;
    float j;
    float k;
    float real;
} GameRotation_t; // 16 bytes

typedef struct __attribute__((packed)) {
    uint32_t       timestamp_ms; // 4 bytes
    BNO_DataType_t type;         // 1 byte
    
    union {
        LinearAccel_t  linear_accel;   // 12 bytes
        GameRotation_t game_rotation;  // 16 bytes
    }; // Union (16 bytes)
} imu_sample_t; // 21 bytes (No padding!)


#define SENS_ON_PIN                 18U
#define IMU_WAKEUP_PIN              7U
#define IMU_LA_SAMPLING_RATE_HZ     5000
#define IMU_GRV_SAMPLING_RATE_HZ    (IMU_LA_SAMPLING_RATE_HZ*5)
#define IMU_ENABLE_GRV              false
#define IMU_ENABLE_LA               true



static uint64_t                 ti=0, te=0;
static uint32_t                 rate=0;
static bno085_handle_t          bno085;


#endif