#include <iostream>
#include "Arduino.h"
#include "bno085.hpp"
#include "LedStrip.hpp"
#include "Battery.hpp"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "lwip/err.h"
#include "lwip/sys.h"


#include "esp_mac.h"
#include "esp_now.h"
#include "esp_crc.h"

#define ESPNOW_MAXDELAY 512

#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_BOTH
#define H2E_IDENTIFIER ""
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_PSK


static EventGroupHandle_t s_wifi_event_group;
static int s_retry_num = 0;

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT  BIT0
#define WIFI_FAIL_BIT       BIT1

#define ESPNOW_WIFI_IF      WIFI_IF_STA

#define LED_PIN 19U
#define LED_ON_PIN 20U
#define SENS_ON_PIN 18U
#define MOTION_WAKEUP_PIN 7U
#define LIGHT_WAKEUP_PIN 5U


void Motion_Init();

// Logs
static const char *MOTION_TAG = "motion_task";
static const char *MAIN = "main";


float _motion_data[23] = { 0.0 };

uint8_t _i2c_write_array[10] = { 0 };
uint8_t _i2c_read_array[10] = { 0 };
uint8_t _i2c_write_size = 0;
void initiator_start();
float x = 0.0;  // X-axis acceleration
float y = 0.0;  // Y-axis acceleration
float z = 0.0;  // Z-axis acceleration

static int64_t start_time, end_time  = 0;  // Start time for motion reading

// Motion, battery and led sensors
static bno085 Motion;
static Battery battery;
static LedStrip led_strip;



