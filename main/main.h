#pragma once

#include <stdio.h>
#include <stddef.h>
#include <stdlib.h>
#include <inttypes.h>
#include "sdkconfig.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/stream_buffer.h"


#include "nvs_flash.h"
#include "driver/gpio.h"
#include "driver/gptimer.h"

#include "esp_timer.h"
#include "esp_log.h"
#include "esp_partition.h"

#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_mac.h"
#include "esp_now.h"
#include "esp_crc.h"
#include "esp_sleep.h"

#include "lwip/err.h"
#include "lwip/sys.h"

#include "espnow.h"
#include "espnow_time.h"
#include "espnow_utils.h"


#include "led_strip.h"
#include "ble_control.h"
#include "imu_flash_log.h"
#include "bno085.h"



// Logs
static const char *TAG = "MAIN";


// IMU
#define SENS_ON_PIN                 18U
#define MOTION_WAKEUP_PIN           7U
#define IMU_LA_SAMPLING_RATE_HZ     5000
#define IMU_GRV_SAMPLING_RATE_HZ    25000

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
} BNO085_FlashLog_t; // 21 bytes (No padding!)


static bno085_handle_t      bno085;





// TIMER
#define GPTIMER_RESOLUTION_HZ       (1000000ULL) // 1 MHz (1 tick = 1 us)
#define TIMESYNC_BLINK_HZ           (3000000ULL)

static bool                 s_timesync_state   = true;
static TaskHandle_t         s_gptimer_task      = NULL;
static gptimer_handle_t     s_gptimer      = NULL;
static QueueHandle_t        s_gptimer_evt_q  = NULL;
static portMUX_TYPE         s_gptimer_lock  = portMUX_INITIALIZER_UNLOCKED;
static uint64_t             gptimer_period=TIMESYNC_BLINK_HZ;
static volatile bool        s_timer_started = false;




// FLASH Log
#define SECTOR_SIZE             4096UL
#define STREAM_BUFFER_SIZE      (SECTOR_SIZE * 4)  // 16KB RAM buffer to absorb flash erase latency



// LED
#define LED_SLP_PIN   20
#define LED_PIN   19                
#define LED_STRIP_NUM_PIXELS 1      
#define BLINKER_MIN_SCHEDULE_AHEAD_US 5000ULL /* 5 ms */

static led_strip_handle_t   s_led_strip    = NULL;
static uint                 gcolor=7;



// ESPNOW TIMESYNC

#define TIMESYNC_BROADCAST_INTERVAL_MS 2000




