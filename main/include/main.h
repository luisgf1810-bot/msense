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
#include "imu.h"




// Logs
const char *TAG = "MAIN";


// TIMER
#define GPTIMER_RESOLUTION_HZ   (1000000ULL) // 1 MHz (1 tick = 1 us)
#define TIMESYNC_BLINK_HZ       (3000000ULL)

static bool                     s_timesync_state   = true;
static TaskHandle_t             s_gptimer_task      = NULL;
static gptimer_handle_t         s_gptimer      = NULL;
static QueueHandle_t            s_gptimer_evt_q  = NULL;
static portMUX_TYPE             s_gptimer_lock  = portMUX_INITIALIZER_UNLOCKED;
static uint64_t                 gptimer_period=TIMESYNC_BLINK_HZ;
static volatile bool            s_timer_started = false;




// LED
#define LED_SLP_PIN   20
#define LED_PIN   19                
#define LED_STRIP_NUM_PIXELS 1      
#define BLINKER_MIN_SCHEDULE_AHEAD_US 5000ULL /* 5 ms */

static led_strip_handle_t   s_led_strip    = NULL;
static uint                 gcolor=7;



// ESPNOW TIMESYNC

#define TIMESYNC_BROADCAST_INTERVAL_MS 2000



// FLASH Log
#define SECTOR_SIZE                 4096UL
#define STREAM_BUFFER_SIZE          (SECTOR_SIZE * 4)  // 16KB RAM buffer to absorb flash erase latency
#define IMU_LOG_PARTITION_LABEL     "imu_log"
#define IMU_SAMPLE_PERIOD_US        10000   /* 10 ms -> 100 Hz */
#define FLASH_SECTOR_SIZE           4096u
#define SECTOR_MAGIC                0x494D5546u   /* "IMUF" */


typedef struct __attribute__((packed)) {
    uint32_t magic;         /* SECTOR_MAGIC when this sector holds valid data */
    uint32_t seq;           /* monotonically increasing write sequence number */
    uint16_t sample_count;  /* number of valid imu_sample_t entries that follow */
    uint16_t reserved;
    uint32_t crc32;         /* CRC32 over the first sample_count samples       */
} sector_header_t;

_Static_assert(sizeof(sector_header_t) == 16, "header must be 16 bytes");

#define SAMPLES_PER_SECTOR ((FLASH_SECTOR_SIZE - sizeof(sector_header_t)) / sizeof(imu_sample_t))

/* A log_sector_t is exactly one flash sector. Sampling writes straight
 * into buf.samples[]; at flush time we finish filling buf.header and
 * push the *entire* 4096-byte struct to flash in a single
 * esp_partition_write() call -- this is the "fastest api" write path:
 * one erase_range() + one write() per sector, no partial writes, no
 * filesystem bookkeeping layered on top. */
typedef struct __attribute__((packed)) {
    sector_header_t             header;                         // 16 byte header
    imu_sample_t                samples[SAMPLES_PER_SECTOR];    // 21 bytes IMU flash
    uint32_t                    padd;                           // 4 byte padding
    uint16_t                    reserved;                       // 2 bytes reserved
} log_sector_t;

_Static_assert(sizeof(log_sector_t) == FLASH_SECTOR_SIZE,  "log_sector_t must be exactly one flash sector");


typedef struct {
    uint32_t sectors_written;
    uint32_t sectors_erase_failed;
    uint32_t sectors_write_failed;
    uint32_t buffer_overruns;     /* writer couldn't keep up in time     */
    uint32_t next_sector;
    uint32_t next_seq;
    uint32_t total_sectors;
    uint32_t wrap_count;          /* how many times the ring has wrapped */
} imu_log_stats_t;




/* Double buffer: while one is being filled by the timer callback, the
 * other is either idle (already flushed) or being written by the
 * writer task. Exactly one of {s_buf[0], s_buf[1]} is "active" at a
 * time; the other is either empty or in flight to flash. */

static portMUX_TYPE             s_mux = portMUX_INITIALIZER_UNLOCKED;
static QueueHandle_t            s_flush_q;      /* holds indices (0/1) of full buffers */
static TaskHandle_t             s_writer_task;
static const esp_partition_t    *s_partition;
static volatile uint8_t         s_active = 0;
static log_sector_t             s_buf[2];

static uint32_t                 s_total_sectors=0;
static uint32_t                 s_next_sector;
static uint32_t                 s_seq;
static imu_log_stats_t          s_stats;
static uint16_t                 ns=0;



