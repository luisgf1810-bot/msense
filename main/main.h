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

// IMU
#define SECTOR_SIZE             4096UL
// 16KB RAM buffer to absorb flash erase latency
#define STREAM_BUFFER_SIZE      (SECTOR_SIZE * 4)  
 // Target 1Hz tracking
#define IMU_SAMPLING_RATE_HZ    1000                
#define SENS_ON_PIN 18U
#define MOTION_WAKEUP_PIN 7U

// 10-byte packed structural representation of one IMU reading 
typedef struct __attribute__((packed)) {
    uint32_t timestamp_us; 
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
} imu_sample_t;

static StreamBufferHandle_t xImuStreamBuffer = NULL;
static bno085_handle_t bno085;
static gptimer_handle_t gptimer = NULL;

// Motion
float _motion_data[23] = { 0.0 };
uint8_t _i2c_write_array[10] = { 0 };
uint8_t _i2c_read_array[10] = { 0 };
uint8_t _i2c_write_size = 0;
float x = 0.0;  // X-axis acceleration
float y = 0.0;  // Y-axis acceleration
float z = 0.0;  // Z-axis acceleration
static int64_t start_time, end_time  = 0;  


// LED
// Driving exactly 1 SK6805 LED
#define LED_SLP_PIN   20
#define LED_GPIO   19                
#define LED_NUM_PIXELS 1      

#define BLINK_PERIOD_US 3000000ULL   /* 3 seconds, in GPTimer ticks (1 tick = 1 us) */
#define BLINK_FLASH_MS  150          /* visible on-time of the flash              */
static led_strip_handle_t led_strip;

#define ON_DELAY_US  (50  * 1000)   // 50 ms ON
#define OFF_DELAY_US (5000 * 1000)  // 5000 ms OFF



// WiFi -  cc:ba:97:f3:34:2c 
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_BOTH
#define H2E_IDENTIFIER ""
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_PSK

static EventGroupHandle_t s_wifi_event_group;
static int s_retry_num = 0;
static bool s_ap_started;
static gptimer_handle_t   s_gptimer      = NULL;
static QueueHandle_t      s_blink_evt_q  = NULL;
static led_strip_handle_t s_led          = NULL;

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT  BIT0
#define WIFI_FAIL_BIT       BIT1
#define ESPNOW_WIFI_IF      WIFI_IF_STA

#define WIFI_SSID       "ESP32_FTM_MASTER"
#define WIFI_PASS       "ftmsync123"
#define WIFI_CHANNEL    6



// Logs
static const char *MOTION_TAG = "MOTION";
static const char *WIFI_TAG = "WIFI";
static const char *ESPNOW_TAG = "ESPNOW";
static const char *ESPNOW_TIMESYNC_TAG = "ESPNOW_TIMESYNC";
static const char *LED_TAG = "LED";
static const char *MAIN_TAG = "MAIN";


// ESPNOW
#define MY_RECEIVER_MAC {0xCC, 0xBA, 0x97, 0xF3, 0x33, 0xE4}
#define ESPNOW_MAXDELAY         512
#define ESPNOW_WIFI_IF          WIFI_IF_STA
#define ESPNOW_QUEUE_SIZE       6
#define IS_BROADCAST_ADDR(addr) (memcmp(addr, s_broadcast_mac, ESP_NOW_ETH_ALEN) == 0)

/*
#undef ESPNOW_INIT_CONFIG_DEFAULT
#define ESPNOW_INIT_CONFIG_DEFAULT()             \
    {                                            \
        .pmk = "ESP_NOW",                        \
        .forward_enable = 1,                     \
        .forward_switch_channel = 0,             \
        .sec_enable = 0,                         \
        .reverse = 0,                            \
        .qsize = 32,                             \
        .send_retry_num = 10,                    \
        .send_max_timeout = pdMS_TO_TICKS(3000), \
        .receive_enable = {                      \
            .ack = 1,                            \
            .forward = 1,                        \
            .group = 1,                          \
            .provisoning = 0,                    \
            .control_bind = 0,                   \
            .control_data = 0,                   \
            .ota_status = 0,                     \
            .ota_data = 0,                       \
            .debug_log = 0,                      \
            .debug_command = 0,                  \
            .data = 0,                           \
            .sec_status = 0,                     \
            .sec = 0,                            \
            .sec_data = 0,                       \
            .reserved = 0,                       \
        },                                       \
    }
*/


enum {
    ESPNOW_DATA_BROADCAST,
    ESPNOW_DATA_UNICAST,
    ESPNOW_DATA_MAX,
};

typedef struct __attribute__((packed)) {
    uint32_t random_value;
    bool button_pushed;
} my_data_t;

static QueueHandle_t s_espnow_queue = NULL;

typedef enum {
    ESPNOW_SEND_CB,
    ESPNOW_RECV_CB,
} espnow_event_id_t;

typedef struct {
    uint8_t mac_addr[ESP_NOW_ETH_ALEN];
    esp_now_send_status_t status;
} espnow_event_send_cb_t;

typedef struct {
    uint8_t mac_addr[ESP_NOW_ETH_ALEN];
    uint8_t *data;
    int data_len;
} espnow_event_recv_cb_t;

typedef union {
    espnow_event_send_cb_t send_cb;
    espnow_event_recv_cb_t recv_cb;
} espnow_event_info_t;

/* When ESPNOW sending or receiving callback function is called, post event to ESPNOW task. */
typedef struct {
    espnow_event_id_t id;
    espnow_event_info_t info;
} espnow_event_t;


/* User defined field of ESPNOW data in this example. */
typedef struct {
    uint8_t type;                         //Broadcast or unicast ESPNOW data.
    uint8_t state;                        //Indicate that if has received broadcast ESPNOW data or not.
    uint16_t seq_num;                     //Sequence number of ESPNOW data.
    uint16_t crc;                         //CRC16 value of ESPNOW data.
    uint32_t magic;                       //Magic number which is used to determine which device to send unicast ESPNOW data.
    uint8_t paylmotion_taskoad[0];                   //Real payload of ESPNOW data.
} __attribute__((packed)) espnow_data_t;

/* Parameters of sending ESPNOW data. */
typedef struct {
    bool unicast;                         //Send unicast ESPNOW data.
    bool broadcast;                       //Send broadcast ESPNOW data.
    uint8_t state;                        //Indicate that if has received broadcast ESPNOW data or not.
    uint32_t magic;                       //Magic number which is used to determine which device to send unicast ESPNOW data.
    uint16_t count;                       //Total count of unicast ESPNOW data to be sent.
    uint16_t delay;                       //Delay between sending two ESPNOW data, unit: ms.
    int len;                              //Length of ESPNOW data to be sent, unit: byte.
    uint8_t *buffer;                      //Buffer pointing to ESPNOW data.
    uint8_t dest_mac[ESP_NOW_ETH_ALEN];   //MAC address of destination device.
} espnow_send_param_t;


// ESPNOW time sync
static int64_t s_time_offset_us = 0;
static int64_t get_synced_time_us(void);
//static uint ledcolorr = 7;
//static uint ledcolorb = 0;


