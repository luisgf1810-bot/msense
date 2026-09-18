#include "main.h"

#define IS_BROADCAST_ADDR(addr) (memcmp(addr, s_broadcast_mac, ESP_NOW_ETH_ALEN) == 0)


/* Compute the number of ticks from "now" (phase_reference_us) to the
 * next BLINKER_PERIOD_US-aligned boundary, guaranteed to be >=
 * BLINKER_MIN_SCHEDULE_AHEAD_US. */
static uint64_t ticks_to_next_boundary(uint64_t phase_now)
{
    uint64_t delay = gptimer_period - (phase_now % gptimer_period);
    if (delay < BLINKER_MIN_SCHEDULE_AHEAD_US) {
        delay += gptimer_period; /* too close - take nextone */
    }
    return delay;
}


void start_imulogs() {
    // stop timesync
    espnow_time_initiator_stop();
    s_timesync_state=false;

    // init flash logging
    ESP_ERROR_CHECK(flash_log_start());

    // start imu logging
    gptimer_period=IMU_LA_SAMPLING_RATE_HZ;
}

void stop_imulogs() {
    // stop flash logging
    ESP_ERROR_CHECK(flash_log_stop());

    // start espnow timesync
    espnow_time_initiator_config_t config = {
        .sync_interval_ms = TIMESYNC_BROADCAST_INTERVAL_MS,  
    };
    espnow_time_initiator_start(&config);
    s_timesync_state=true;

    // start led blinking
    gptimer_period=TIMESYNC_BLINK_HZ;
}


/* --- GPTimer Init and ISR Callback --- */
static bool IRAM_ATTR timer_alarm_cb(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata,  void *user_ctx) {
    
    BaseType_t high_task_wakeup = pdFALSE;
    
    uint8_t evt = 1;
    xQueueSendFromISR(s_gptimer_evt_q, &evt, &high_task_wakeup);
    return high_task_wakeup == pdTRUE;
}

esp_err_t init_gptimer(uint64_t phase_now) {

    if (s_gptimer != NULL) {
        return ESP_ERR_INVALID_STATE; 
    }

    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT, 
        .direction = GPTIMER_COUNT_UP,          // Monotonic upward counting
        .resolution_hz = GPTIMER_RESOLUTION_HZ,   // 1 MHz resolution = 1 tick per microsecond
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &s_gptimer));


    gptimer_event_callbacks_t cbs = {
        .on_alarm = timer_alarm_cb,
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(s_gptimer, &cbs, NULL));
    ESP_ERROR_CHECK(gptimer_enable(s_gptimer));

    gptimer_alarm_config_t alarm_config = {
        .alarm_count = ticks_to_next_boundary(phase_now),            
        .flags.auto_reload_on_alarm = false 
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(s_gptimer, &alarm_config));
    ESP_ERROR_CHECK(gptimer_start(s_gptimer));

    ESP_LOGI(TAG, "GPTimer started, first alarm in %llu us", (unsigned long long)alarm_config.alarm_count);

    return ESP_OK;
}

esp_err_t gptimer_arm_next(uint64_t phase_now)
{
    if (s_gptimer == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    portENTER_CRITICAL(s_gptimer_lock);
    uint64_t current_raw = 0;
    ESP_ERROR_CHECK(gptimer_get_raw_count(s_gptimer, &current_raw));
    gptimer_alarm_config_t alarm_config = {
        .alarm_count = current_raw + ticks_to_next_boundary(phase_now),
        .reload_count = 0, 
        .flags.auto_reload_on_alarm = false,
    };
    esp_err_t err = gptimer_set_alarm_action(s_gptimer, &alarm_config);
    portEXIT_CRITICAL(s_gptimer_lock);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "failed to arm next alarm: %s", esp_err_to_name(err));
    }

    return err;
}

static void timer_task(void *arg)
{
    uint64_t tick;
    for (;;) {
        if (xQueueReceive(s_gptimer_evt_q, &tick, portMAX_DELAY) == pdTRUE) {  
            
            if (!s_timesync_state) {
                bno085_service(bno085);
            } else {
                led_strip_set_pixel(s_led_strip, 0, 0, gcolor, 0); 
                led_strip_refresh(s_led_strip);
                vTaskDelay(80);
                led_strip_clear(s_led_strip);
            } 

            uint64_t now = (uint64_t)esp_timer_get_time();

            if (s_timesync_state) {
                uint64_t tick = now / gptimer_period;
                ESP_LOGI(TAG, "TICK %" PRIu64 "  synced_t = %" PRIu64 " us", tick, now);
            }

            /* Re-arm the next one-shot alarm right away - the timer
             * never auto-reloads, so this is the only thing keeping it
             * running every 3s. */
            ESP_ERROR_CHECK(gptimer_arm_next(now));
        }
    }
}


/* Initialize led strip */
esp_err_t init_led(void) {

    // Enable the power supply to the LED Strip 
    gpio_set_direction(LED_SLP_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_SLP_PIN, 1);

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
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &s_led_strip));
    led_strip_clear(s_led_strip);

    ESP_LOGI(TAG, "LED initialized"); 

    return ESP_OK;
}



/* --- Initialize Wi-Fi & ESP-NOW TIME Sync --- */
esp_err_t init_espnow_timesync(void) {

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK( esp_wifi_start());
    ESP_ERROR_CHECK( esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE));
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

    espnow_config_t espnow_config = ESPNOW_INIT_CONFIG_DEFAULT();
    espnow_config.qsize = 32;
    ESP_ERROR_CHECK( espnow_init(&espnow_config) );


    // Start as time initiator (controller)
    espnow_time_initiator_config_t config = {
        .sync_interval_ms = TIMESYNC_BROADCAST_INTERVAL_MS,  
    };
    espnow_time_initiator_start(&config);

    ESP_LOGI(TAG, "ESPNOW TIMESYNC initialized"); 

    return ESP_OK;
}



/* --- Initialize and IMU callbacks --- */
static void on_sensor_data(bno085_handle_t handle, const bno085_sensor_value_t *value, void *ctx)
{
    switch (value->sensor_id) {

        case BNO085_SENSOR_GAME_ROTATION_VECTOR:
            printf("%.4f,%.4f,%.4f,%.4f\n",
                    value->data.game_rotation_vector.i, 
                    value->data.game_rotation_vector.j,
                    value->data.game_rotation_vector.k,
                    value->data.game_rotation_vector.real
                    );
            break;
        case BNO085_SENSOR_LINEAR_ACCELERATION:
            printf("%.4f,%.4f,%.4f\n",
                    value->data.linear_acceleration.x, 
                    value->data.linear_acceleration.y,
                    value->data.linear_acceleration.z
                    );
            break;

        default:
            break;
               
    }
}

esp_err_t imu_init() {

    // Create I2C bus for BNO085
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

    // Create I2C device for BNO085
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = 0x4A,  // AD0 = GND
        .scl_speed_hz = 400000,
    };
    i2c_master_dev_handle_t i2c_dev;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_config, &i2c_dev));

    // Initialize BNO085
    ESP_ERROR_CHECK(bno085_init(NULL, i2c_dev, GPIO_NUM_7, GPIO_NUM_18, &bno085));  
    bno085_register_sensor_callback(bno085, on_sensor_data, NULL);
    bno085_enable_sensor(bno085, BNO085_SENSOR_LINEAR_ACCELERATION, IMU_LA_SAMPLING_RATE_HZ);  // 200hz
    bno085_enable_sensor(bno085, BNO085_SENSOR_GAME_ROTATION_VECTOR, IMU_GRV_SAMPLING_RATE_HZ); // 40hz

    ESP_LOGI(TAG, "IMU initialized, GRV:%d hz, LA:%d hz", IMU_GRV_SAMPLING_RATE_HZ, IMU_LA_SAMPLING_RATE_HZ ); 

    return ESP_OK;
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

    // blink stuff
    s_gptimer_evt_q = xQueueCreate(4, sizeof(uint64_t));
    xTaskCreate(timer_task, "timer_task", 4096, NULL, 5, &s_gptimer_task);  

    // SetUp
    //ESP_ERROR_CHECK(battery.Init());
    ESP_ERROR_CHECK(init_led());
    ESP_ERROR_CHECK(init_espnow_timesync());
    ESP_ERROR_CHECK(init_gptimer(esp_timer_get_time()));
    ESP_ERROR_CHECK(imu_init());
    ESP_ERROR_CHECK(ble_control_init());

    ESP_LOGI(TAG, "Master ready - broadcasting every %d ms, blinking every %llu us",
             TIMESYNC_BROADCAST_INTERVAL_MS, gptimer_period);
}
