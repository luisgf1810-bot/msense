#include "main.h"


// Setup GPIOs
void SetupPins() {
    // Enable the power supply to the LED Strip 
    gpio_set_direction(LED_SLP_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_SLP_PIN, 1);

}


static bool IRAM_ATTR led_timer_alarm_cb(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx)
{
    BaseType_t hp_task_woken = pdFALSE;
    uint64_t tick = edata->alarm_value;
    xQueueSendFromISR(s_blink_evt_q, &tick, &hp_task_woken);
    return hp_task_woken == pdTRUE;
}

static bool IRAM_ATTR imu_timer_alarm_cb(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx) {
    BaseType_t high_task_awoken = pdFALSE;
    
    
    // Return true if a high-priority task was awakened to trigger a context switch
    return high_task_awoken == pdTRUE;
}

void init_timers() {

    // create IMU timer
    gptimer_config_t imu_timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1 * 1000 * 1000, 
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&imu_timer_config, &s_gptimer_imu));

    // register callback for IMU timer
    gptimer_event_callbacks_t imu_cbs = {
        .on_alarm = imu_timer_alarm_cb,
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(s_gptimer_imu, &imu_cbs, NULL));

    // create alarm for IMU timer
    gptimer_alarm_config_t imu_alarm_config = {
        .alarm_count = 1000000, 
        .flags.auto_reload_on_alarm = true,
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(s_gptimer_imu, &imu_alarm_config));
    ESP_ERROR_CHECK(gptimer_enable(s_gptimer_imu));
    ESP_ERROR_CHECK(gptimer_start(s_gptimer_imu));


    // create LED timer
    gptimer_config_t led_timer_cfg = {
        .clk_src      = GPTIMER_CLK_SRC_DEFAULT,
        .direction    = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000,   /* 1 tick = 1 us */
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&led_timer_cfg, &s_gptimer_led));

    // register callback for LED timer
    gptimer_event_callbacks_t led_cbs = { 
        .on_alarm = led_timer_alarm_cb 
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(s_gptimer_led, &led_cbs, NULL));

    // create alarm for LED timer
    gptimer_alarm_config_t led_alarm_config = {
        .alarm_count = 3000000,  /* 3 seconds */
        .flags.auto_reload_on_alarm = true,
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(s_gptimer_led, &led_alarm_config));
    ESP_ERROR_CHECK(gptimer_enable(s_gptimer_led));
    ESP_ERROR_CHECK(gptimer_start(s_gptimer_led));
}


// Led
void led_init() {

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

static void blink_task(void *arg)
{
    uint64_t tick;
    for (;;) {
        if (xQueueReceive(s_blink_evt_q, &tick, portMAX_DELAY) == pdTRUE) {
            ESP_LOGI(LED_TAG, "blink @ t=%llu us (reference clock)", (unsigned long long)tick);
            led_strip_set_pixel(s_led, 0, 0, 0, 7);   /* green flash */
            led_strip_refresh(s_led);
            vTaskDelay(pdMS_TO_TICKS(BLINK_FLASH_MS));
            led_strip_clear(s_led);
        }
    }
}



// Wifi 
static void wifi_sta_init()
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK( esp_wifi_start());
    ESP_ERROR_CHECK( esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE));

}






static void timesync_event_handler(void *arg, esp_event_base_t base, int32_t event_id, void *event_data)
{
    switch (event_id) {
        case ESP_EVENT_ESPNOW_TIMESYNC_STARTED:
            ESP_LOGI(ESPNOW_TIMESYNC_TAG, "Time sync started");
            break;
        case ESP_EVENT_ESPNOW_TIMESYNC_STOPPED:
            ESP_LOGI(ESPNOW_TIMESYNC_TAG, "Time sync stopped");
            break;
        case ESP_EVENT_ESPNOW_TIMESYNC_TIMEOUT:
            ESP_LOGW(ESPNOW_TIMESYNC_TAG, "Time sync timeout");
            break;
        default:
            break;
    }
}

void espnow_close()
{
    esp_now_deinit();
}

void espnow_start(void) {

    espnow_config_t espnow_config = ESPNOW_INIT_CONFIG_DEFAULT();
    espnow_config.qsize = 32;
    esp_event_handler_register(ESP_EVENT_ESPNOW, ESP_EVENT_ANY_ID, timesync_event_handler, NULL);
    ESP_ERROR_CHECK( espnow_init(&espnow_config) );

    // Start as time initiator (controller)
    espnow_time_initiator_config_t config = {
        .sync_interval_ms = 10000,  // 10 seconds
    };
    espnow_time_initiator_start(&config);
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




// Main
void Initialize() {

    // Init GIOs
    SetupPins();
    ESP_LOGI(MAIN_TAG, "GPIO pins initialized");

    // Battery init
    //battery.Init();

    // Timers
    init_timers();

    // FLASH Log init
    ESP_ERROR_CHECK(imu_flash_log_init());
    ESP_LOGI(MAIN_TAG, "IMU flash initialized");

    // IMU init
    imu_init();
    ESP_LOGI(MAIN_TAG, "BNO085 and timer initialized");

    // BLE control init
    ESP_ERROR_CHECK(ble_control_init());
    ESP_LOGI(MAIN_TAG, "BLE control initialized");

    // Init led
    led_init();

    // WiFi init
    wifi_sta_init();

    // Espnow init
    espnow_start();



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

    xTaskCreate(blink_task, "blink_task", 4096, NULL, 5, NULL);  

}



