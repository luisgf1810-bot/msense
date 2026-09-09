#include "main.h"



/* --- GPTimer ISR Callback --- */
static bool IRAM_ATTR gptimer_on_alarm_cb(gptimer_handle_t timer,   const gptimer_alarm_event_data_t *edata,  void *user_ctx) {
    
    BaseType_t high_task_wakeup = pdFALSE;
    uint8_t evt = 1;
    xQueueSendFromISR(s_blink_evt_q, &evt, &high_task_wakeup);
    return high_task_wakeup == pdTRUE;
}

static bool IRAM_ATTR imu_timer_alarm_cb(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx) {
    BaseType_t high_task_awoken = pdFALSE;
    
    
    // Return true if a high-priority task was awakened to trigger a context switch
    return high_task_awoken == pdTRUE;
}


/* --- LED Toggle --- */
void blinker_led_toggle(void)
{
    if (!s_led) {
        return;
    }

    led_state = !led_state;
    if (led_state) {
        led_strip_set_pixel(s_led, 0, 0, 7, 0); /* dim green */
        led_strip_refresh(s_led);
    } else {
        led_strip_clear(s_led);
    }
}

static void blink_task(void *arg)
{
    uint64_t tick;
    for (;;) {
        if (xQueueReceive(s_blink_evt_q, &tick, portMAX_DELAY) == pdTRUE) {
            blinker_led_toggle();
            ESP_LOGI(LED_TAG, "blink @ t = %lld us (reference clock)", (long long)esp_timer_get_time());
            
        }
    }
}




// Setup GPIOs
void SetupPins() {
    // Enable the power supply to the LED Strip 
    gpio_set_direction(LED_SLP_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_SLP_PIN, 1);

}

/* --- Initialize Addressable LED --- */
static void init_led(void) {

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

    ESP_LOGI(LED_TAG, "LED initialized"); 

}


/* --- Initialize Wi-Fi & ESP-NOW Managed Sync --- */
static void init_espnow_timesync(void) {

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
        .sync_interval_ms = TIMESYNC_BROADCAST_INTERVAL_MS,  // 5 seconds
    };
    espnow_time_initiator_start(&config);
}


/* --- Initialize Hardware GPTimer --- */
static void init_gptimer(uint64_t phase_reference_us) {
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = TIMER_RESOLUTION_HZ,
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &s_gptimer_led));

    gptimer_event_callbacks_t cbs = {
        .on_alarm = gptimer_on_alarm_cb,
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(s_gptimer_led, &cbs, NULL));
    ESP_ERROR_CHECK(gptimer_enable(s_gptimer_led));

    /* Pre-load the raw counter with our current position inside the
     * 3-second cycle. The alarm is fixed at BLINKER_PERIOD_US, so the
     * very first alarm fires after exactly (BLINKER_PERIOD_US - phase)
     * ticks - i.e. precisely on the next aligned boundary. After that,
     * auto-reload-to-0 keeps every subsequent alarm exactly
     * BLINKER_PERIOD_US ticks apart. */
    uint64_t phase = phase_reference_us % BLINK_PERIOD_US;
    ESP_ERROR_CHECK(gptimer_set_raw_count(s_gptimer_led, phase));

    gptimer_alarm_config_t alarm_config = {
        .reload_count = 0,
        .alarm_count = BLINK_PERIOD_US,
        .flags.auto_reload_on_alarm = true,
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(s_gptimer_led, &alarm_config));
    ESP_ERROR_CHECK(gptimer_start(s_gptimer_led));

    ESP_LOGI(MAIN_TAG, "GPTimer started, initial phase = %llu us into the 3s cycle", (unsigned long long)phase);
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

void imu_init() {
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
    bno085_enable_sensor(bno085, BNO085_SENSOR_LINEAR_ACCELERATION, 100000);  // 10Hz

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

    // Init GIOs
    SetupPins();
    ESP_LOGI(MAIN_TAG, "GPIO pins initialized");

    // Battery init
    //battery.Init();

    // Init led
    init_led();

    // Init ESP-NOW and time sync
    init_espnow_timesync();

    // Timers
    init_gptimer((uint64_t)esp_timer_get_time());

    // FLASH Log init
    ESP_ERROR_CHECK(imu_flash_log_init());
    ESP_LOGI(MAIN_TAG, "IMU flash initialized");

    // IMU init
    imu_init();
    ESP_LOGI(MAIN_TAG, "BNO085 and timer initialized");

    // BLE control init
    ESP_ERROR_CHECK(ble_control_init());
    ESP_LOGI(MAIN_TAG, "BLE control initialized");

    xTaskCreate(blink_task, "blink_task", 4096, NULL, 5, NULL);  

    ESP_LOGI(MAIN_TAG, "Master ready - broadcasting time every %d ms, blinking every %llu us",
             TIMESYNC_BROADCAST_INTERVAL_MS, (unsigned long long)BLINK_PERIOD_US);
}




