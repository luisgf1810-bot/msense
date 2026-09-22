#ifndef IMU_STRUCT_H
#define IMU_STRUCT_H

#include <inttypes.h>

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


#endif