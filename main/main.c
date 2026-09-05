#include "main.h"


// Setup GPIOs
void SetupPins() {
    // Enable the power supply to the LED Strip 
    gpio_set_direction(LED_SLP_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_SLP_PIN, 1);

}




// Led
void led_init() {

    /* Enable the power supply to the LED Strip */
    gpio_set_direction(LED_SLP_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_SLP_PIN, 1);

    led_rcolor = 7;
    led_gcolor = 0;


    led_strip_config_t strip_config = {
        .strip_gpio_num = LED_PIN,
        .max_leds = LED_STRIP_NUM_PIXELS,
        .led_model = LED_MODEL_SK6812, // SK6805 shares close timing with SK6812/WS2812
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
        .flags.invert_out = false,
    };
    led_strip_rmt_config_t rmt_config = {
        .clk_src       = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000,
        .flags.with_dma = false,
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &s_led));
    led_strip_clear(s_led);
}

static bool IRAM_ATTR on_gptimer_alarm(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx)
{
    BaseType_t hp_task_woken = pdFALSE;

    uint64_t next_target = edata->alarm_value + BLINK_PERIOD_US;
    gptimer_alarm_config_t alarm_cfg = {
        .alarm_count = next_target,
        .reload_count = 0,
        .flags.auto_reload_on_alarm = false,
    };
    /* Re-arming a manual-reload alarm from inside its own callback is the
     * documented ESP-IDF gptimer pattern for "one-shot, re-armed each time". */
    gptimer_set_alarm_action(timer, &alarm_cfg);

    uint64_t tick = edata->alarm_value;
    xQueueSendFromISR(s_blink_evt_q, &tick, &hp_task_woken);
    return hp_task_woken == pdTRUE;
}

static void blink_task(void *arg)
{
    uint64_t tick;
    for (;;) {
        if (xQueueReceive(s_blink_evt_q, &tick, portMAX_DELAY) == pdTRUE) {
            ESP_LOGI(TAG, "blink @ t=%llu us (reference clock)", (unsigned long long)tick);
            led_strip_set_pixel(s_led, 0, led_rcolor, led_gcolor, 0);   /* green flash */
            led_strip_refresh(s_led);
            vTaskDelay(pdMS_TO_TICKS(BLINK_FLASH_MS));
            led_strip_clear(s_led);
        }
    }
}



// Wifi 
static void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
     if (event_id == WIFI_EVENT_FTM_REPORT) {
        /*wifi_event_ftm_report_t *event = (wifi_event_ftm_report_t *) event_data;
        s_rtt_est = event->rtt_est;
        s_dist_est = event->dist_est;
        s_ftm_report_num_entries = event->ftm_report_num_entries;
        if (event->status == FTM_STATUS_SUCCESS) {
            xEventGroupSetBits(s_ftm_event_group, FTM_REPORT_BIT);
        } else if (event->status == FTM_STATUS_USER_TERM) {
            ESP_LOGI(TAG_STA, "User terminated FTM procedure");
        } else {
            ESP_LOGI(TAG_STA, "FTM procedure with Peer("MACSTR") failed! (Status - %d)",
                     MAC2STR(event->peer_mac), event->status);
            xEventGroupSetBits(s_ftm_event_group, FTM_FAILURE_BIT);
        }*/
    } else if (event_id == WIFI_EVENT_AP_START) {
        s_ap_started = true;
    } else if (event_id == WIFI_EVENT_AP_STOP) {
        s_ap_started = false;
    }
}

static void wifi_softap_init()
{

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t ap_config = {
        .ap = {
            .ssid          = WIFI_SSID,
            .ssid_len      = strlen(WIFI_SSID),
            .channel       = WIFI_CHANNEL,
            .password      = WIFI_PASS,
            .max_connection = 4,
            .authmode      = WIFI_AUTH_WPA2_PSK,
            .ftm_responder = true,      /* <-- serves FTM requests, esp_wifi_types_generic.h */
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    /* Keep the AP on 20 MHz bandwidth - Espressif's own FTM example notes
     * this gives measurably more accurate FTM timing than 40 MHz. */
    esp_wifi_set_bandwidth(WIFI_IF_AP, WIFI_BW20);

    ESP_LOGI(TAG, "SoftAP '%s' up on channel %d, FTM responder enabled", WIFI_SSID, WIFI_CHANNEL);
}

static void gptimer_init_from_tsf(void)
{
    /* Seed the GPTimer from this radio's own TSF, then let it free-run. */
    gptimer_config_t timer_cfg = {
        .clk_src      = GPTIMER_CLK_SRC_DEFAULT,
        .direction    = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000,   /* 1 tick = 1 us */
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_cfg, &s_gptimer));

    gptimer_event_callbacks_t cbs = { .on_alarm = on_gptimer_alarm };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(s_gptimer, &cbs, NULL));
    ESP_ERROR_CHECK(gptimer_enable(s_gptimer));

    int64_t tsf_now = esp_wifi_get_tsf_time(WIFI_IF_AP);
    ESP_ERROR_CHECK(gptimer_set_raw_count(s_gptimer, (uint64_t)tsf_now));

    uint64_t first_target = (((uint64_t)tsf_now / BLINK_PERIOD_US) + 1) * BLINK_PERIOD_US;
    gptimer_alarm_config_t alarm_cfg = {
        .alarm_count = first_target,
        .reload_count = 0,
        .flags.auto_reload_on_alarm = false,
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(s_gptimer, &alarm_cfg));
    ESP_ERROR_CHECK(gptimer_start(s_gptimer));

    ESP_LOGI(MAIN_TAG, "GPTimer seeded from AP TSF=%lld us", (long long)tsf_now);
}



// ESPNOW time sync
static int64_t get_synced_time_us(void)
{
    return esp_timer_get_time() ;
}

void espnow_timesync_init() {
    // Start as time initiator (controller)
    espnow_time_initiator_config_t config = {
        .sync_interval_ms = 30000,  // Broadcast time every 30 seconds
    };
    espnow_time_initiator_start(&config);
    ESP_LOGI(ESPNOW_TIMESYNC_TAG, "Time sync initiator started, broadcast rate: %d ms", config.sync_interval_ms);
}



// Espnow
int espnow_data_parse(uint8_t *data, uint16_t data_len, uint8_t *state, uint16_t *seq, uint32_t *magic)
{
    espnow_data_t *buf = (espnow_data_t *)data;
    uint16_t crc, crc_cal = 0;

    if (data_len < sizeof(espnow_data_t)) {
        ESP_LOGE(ESPNOW_TAG, "Receive ESPNOW data too short, len:%d", data_len);
        return -1;
    }

    *state = buf->state;
    *seq = buf->seq_num;
    *magic = buf->magic;
    crc = buf->crc;
    buf->crc = 0;
    crc_cal = esp_crc16_le(UINT16_MAX, (uint8_t const *)buf, data_len);

    if (crc_cal == crc) {
        return buf->type;
    }

    return -1;
}

static void espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
{
    espnow_event_t evt;
    espnow_event_recv_cb_t *recv_cb = &evt.info.recv_cb;
    uint8_t * mac_addr = recv_info->src_addr;
    uint8_t * des_addr = recv_info->des_addr;

    if (mac_addr == NULL || data == NULL || len <= 0) {
        ESP_LOGE(ESPNOW_TAG, "Receive cb arg error");
        return;
    }

    evt.id = ESPNOW_RECV_CB;
    memcpy(recv_cb->mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
    recv_cb->data = (uint8_t *)malloc(len);
    if (recv_cb->data == NULL) {
        ESP_LOGE(ESPNOW_TAG, "Malloc receive data fail");
        return;
    }
    memcpy(recv_cb->data, data, len);
    recv_cb->data_len = len;
    if (xQueueSend(s_espnow_queue, &evt, ESPNOW_MAXDELAY) != pdTRUE) {
        ESP_LOGW(ESPNOW_TAG, "Send receive queue fail");
        free(recv_cb->data);
    }
}

static void espnow_task(void *p)
{
    espnow_event_t evt;
    int ret;
    uint8_t recv_state = 0;
    uint16_t recv_seq = 0;
    uint32_t recv_magic = 0;

    while (xQueueReceive(s_espnow_queue, &evt, portMAX_DELAY) == pdTRUE) {

        switch (evt.id) {
            case ESPNOW_RECV_CB:
            {
                espnow_event_recv_cb_t *recv_cb = &evt.info.recv_cb;
                ret = espnow_data_parse(recv_cb->data, recv_cb->data_len, &recv_state, &recv_seq, &recv_magic);
                free(recv_cb->data);

                if (ret == ESPNOW_DATA_BROADCAST) {
                    ESP_LOGI(ESPNOW_TAG, "Receive %dth broadcast data from: " MACSTR "", recv_seq, MAC2STR(recv_cb->mac_addr), recv_cb->data_len);

                }
                else if (ret == ESPNOW_DATA_UNICAST) {
                    ESP_LOGI(ESPNOW_TAG, "Receive %dth unicast data from: " MACSTR ", len: %d", recv_seq, MAC2STR(recv_cb->mac_addr), recv_cb->data_len);

                }
                else {
                    ESP_LOGI(ESPNOW_TAG, "Receive error data from: " MACSTR "", MAC2STR(recv_cb->mac_addr));
                }
                break;
            }
            default:
                ESP_LOGE(ESPNOW_TAG, "Callback type error: %d", evt.id);
                break;
        }
    }
}

void espnow_close(espnow_send_param_t *send_param)
{
    vQueueDelete(s_espnow_queue);
    s_espnow_queue = NULL;
    esp_now_deinit();
}

void espnow_start() {

    s_espnow_queue = xQueueCreate(ESPNOW_QUEUE_SIZE, sizeof(espnow_event_t));
    if (s_espnow_queue == NULL) {
        ESP_LOGE(ESPNOW_TAG, "Create queue fail");
        esp_now_deinit();
        return;
    }

    espnow_config_t espnow_config = ESPNOW_INIT_CONFIG_DEFAULT();
    espnow_config.qsize = CONFIG_APP_ESPNOW_QUEUE_SIZE;
    ESP_ERROR_CHECK( espnow_init(&espnow_config) );

    //xTaskCreate(espnow_task, "espnow_task", 2048, NULL, 4, NULL);
}




// IMU
static void on_sensor_data(bno085_handle_t handle, const bno085_sensor_value_t *value, void *ctx)
{
    if (value->sensor_id == BNO085_SENSOR_LINEAR_ACCELERATION) {
        ESP_LOGI(MAIN_TAG, "(%" PRIu64 ") Linear Acceleration: x=%.4f, y=%.4f, z=%.4f", esp_timer_get_time(),
               value->data.linear_acceleration.x, value->data.linear_acceleration.y,
               value->data.linear_acceleration.z);
    }
}

static bool imu_timer_alarm_cb(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx) {
    BaseType_t high_task_awoken = pdFALSE;
    
    
    // Return true if a high-priority task was awakened to trigger a context switch
    return high_task_awoken == pdTRUE;
}

void imu_init() {
    // IMU chip
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = GPIO_NUM_8,
        .scl_io_num = GPIO_NUM_9,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus_handle));

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = 0x4A,  // AD0 = GND
        .scl_speed_hz = 400000,
    };
    i2c_master_dev_handle_t i2c_dev;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_config, &i2c_dev));

    ESP_ERROR_CHECK(bno085_init(NULL, i2c_dev, GPIO_NUM_7, GPIO_NUM_18, &bno085));  // NULL = default config
    bno085_register_sensor_callback(bno085, on_sensor_data, NULL);
    bno085_enable_sensor(bno085, BNO085_SENSOR_LINEAR_ACCELERATION, 100000);  // 10Hz


    // Sampling gtimer
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1 * 1000 * 1000, 
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer));

    // Register the alarm callback function
    gptimer_event_callbacks_t cbs = {
        .on_alarm = imu_timer_alarm_cb,
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer, &cbs, NULL));

    // Set alarm period (1,000,000 ticks = 1 Hz sampling rate)
    gptimer_alarm_config_t alarm_config = {
        .alarm_count = 1000000, 
        .flags.auto_reload_on_alarm = true,
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer, &alarm_config));

    // Enable and start the hardware timer
    ESP_ERROR_CHECK(gptimer_enable(gptimer));
    ESP_ERROR_CHECK(gptimer_start(gptimer));


}




// Main
void Initialize() {

    // Init GIOs
    SetupPins();
    ESP_LOGI(MAIN_TAG, "GPIO pins initialized");

      // Battery init
    //battery.Init();

    // FLASH Log init
    ESP_ERROR_CHECK(imu_flash_log_init());
    ESP_LOGI(MAIN_TAG, "IMU flash initialized");

    // IMU init
    imu_init();
    ESP_LOGI(MAIN_TAG, "BNO085 and timer initialized");

    // BLE control init
    ESP_ERROR_CHECK(ble_control_init());
    ESP_LOGI(MAIN_TAG, "BLE control initialized");

    // Wifi and hw timer from FTM
    wifi_softap_init();
    gptimer_init_from_tsf();

    // Init led
    led_init();
    ESP_LOGI(MAIN_TAG, "LED initialized"); 
}




// App main
void app_main()
{
     // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    s_blink_evt_q = xQueueCreate(4, sizeof(uint64_t));

    // Init components
    Initialize();

    xTaskCreate(blink_task, "blink_task", 4096, NULL, 10, NULL);  

}



