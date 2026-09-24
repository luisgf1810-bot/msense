#include "main.h"

#define IS_BROADCAST_ADDR(addr) (memcmp(addr, s_broadcast_mac, ESP_NOW_ETH_ALEN) == 0)





/*  Battery */

esp_err_t init_battery() {
  
    return ESP_OK;
}




/* --- Flash functions --- */

static inline bool seq_is_newer(uint32_t a, uint32_t b)
{
    return (int32_t)(a - b) > 0;
}
                                         
esp_err_t init_flash(void) {

    s_partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, 0x40, IMU_LOG_PARTITION_LABEL);
    if (!s_partition) {
        ESP_LOGE(TAG, "partition '%s' not found -- check partitions.csv", IMU_LOG_PARTITION_LABEL);
        return ESP_ERR_NOT_FOUND;
    }
    if (s_partition->size % FLASH_SECTOR_SIZE != 0) {
        ESP_LOGE(TAG, "partition size must be a multiple of %u bytes", FLASH_SECTOR_SIZE);
        return ESP_ERR_INVALID_SIZE;
    }

    s_total_sectors = (uint32_t)(s_partition->size / FLASH_SECTOR_SIZE);
    memset(s_buf, 0, sizeof(s_buf));
    memset(&s_stats, 0, sizeof(s_stats));
    s_stats.total_sectors = s_total_sectors;

    ESP_LOGI(TAG, "partition '%s': %u bytes, %" PRIu32 " sectors, %u samples/sector",
                s_partition->label, (unsigned)s_partition->size, s_total_sectors,
                (unsigned)SAMPLES_PER_SECTOR);

    return ESP_OK;
}

esp_err_t flash_log_start(void)
{
   
    s_next_sector = 0;
    s_seq = 0;
    s_stats.next_sector = s_next_sector;
    s_stats.next_seq    = s_seq;

    return ESP_OK;
}

esp_err_t flash_log_stop(void) {
    ESP_LOGI(TAG, "flashlog stoped");
    return ESP_OK;
}

esp_err_t imu_flash_log_flush_partial(void)
{
    uint8_t idx;
    uint16_t count;

    taskENTER_CRITICAL(&s_mux);
    idx = s_active;
    count = s_buf[idx].header.sample_count;
    if (count > 0) {
        uint8_t other = 1 - idx;
        s_active = other; /* stop new samples from landing in idx */
    }
    taskEXIT_CRITICAL(&s_mux);

    if (count == 0) {
        return ESP_OK; /* nothing pending */
    }
    xQueueSend(s_flush_q, &idx, portMAX_DELAY);
    return ESP_OK;
}

void imu_flash_log_get_stats(imu_log_stats_t *out)
{
    if (!out) return;
    taskENTER_CRITICAL(&s_mux);
    *out = s_stats;
    taskEXIT_CRITICAL(&s_mux);
}

esp_err_t imu_flash_log_read_sector_raw(uint32_t sector_index, void *out_buf_4096_bytes)
{
    if (sector_index >= s_total_sectors) {
        return ESP_ERR_INVALID_ARG;
    }
    return esp_partition_read(s_partition, (size_t)sector_index * FLASH_SECTOR_SIZE,  out_buf_4096_bytes, FLASH_SECTOR_SIZE);
}

static void write_sector_to_flash(log_sector_t *sec)
{
    const uint8_t *payload = (const uint8_t *)sec + sizeof(sector_header_t);

    sec->header.magic = SECTOR_MAGIC;
    sec->header.seq   = s_seq++;
    sec->header.crc32 = esp_rom_crc32_le(0, payload, (uint32_t)sec->header.sample_count * sizeof(imu_sample_t));

    size_t offset = (size_t)s_next_sector * FLASH_SECTOR_SIZE;

    int64_t t0 = esp_timer_get_time();

    /* Flash can only clear bits via erase; every sector must be erased
     * before it is reused (this is a ring, so after the first lap every
     * sector already holds old data). */
    esp_err_t err = esp_partition_erase_range(s_partition, offset, FLASH_SECTOR_SIZE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "erase failed @ sector %" PRIu32 ": %s", s_next_sector, esp_err_to_name(err));
        s_stats.sectors_erase_failed++;
        goto advance;
    }

    /* Single write call for the whole sector (header + payload together)
     * -- this is the fastest available IDF path for raw partition I/O:
     * esp_partition_write() maps directly onto the underlying
     * spi_flash_write(), with no filesystem indirection. */
    err = esp_partition_write(s_partition, offset, sec, FLASH_SECTOR_SIZE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "write failed @ sector %" PRIu32 ": %s", s_next_sector, esp_err_to_name(err));
        s_stats.sectors_write_failed++;
        goto advance;
    }

    s_stats.sectors_written++;
    ESP_LOGI(TAG, "sector %" PRIu32 " (seq %" PRIu32 ", %u samples) written in %lld us",
              s_next_sector, 
              sec->header.seq, 
              sec->header.sample_count,
              (long long)(esp_timer_get_time() - t0)
    );

advance:
    s_next_sector++;
    if (s_next_sector >= s_total_sectors) {
        stop_imulogs();
        /*s_next_sector = 0;
        s_stats.wrap_count++;*/
    }
    s_stats.next_sector     = s_next_sector;
    s_stats.next_seq        = s_seq;
}

static void flash_task(void *arg)
{
    uint8_t idx;
    for (;;) {
        if (xQueueReceive(s_flush_q, &idx, portMAX_DELAY) == pdTRUE) {
            write_sector_to_flash(&s_buf[idx]);
            /* Buffer is now free for the sampler to reuse. */
            taskENTER_CRITICAL(&s_mux);
            s_buf[idx].header.sample_count = 0;
            taskEXIT_CRITICAL(&s_mux);
        }
    }
}




/* --- GPTimer Init and ISR Callback --- */

static uint64_t ticks_to_next_boundary(uint64_t phase_now)
{
    uint64_t delay = gptimer_period - (phase_now % gptimer_period);
    if (delay < BLINKER_MIN_SCHEDULE_AHEAD_US) {
        delay += gptimer_period; /* too close - take nextone */
    }
    return delay;
}

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



/* --- IMU  --- */
static void on_sensor_data(bno085_handle_t handle, const bno085_sensor_value_t *value, void *ctx)
{

    uint8_t full_idx = 0xFF;
    imu_sample_t sample;

    sample.timestamp_ms = (uint32_t)(esp_timer_get_time()/1000);
    
    switch (value->sensor_id) {

        case BNO085_SENSOR_GAME_ROTATION_VECTOR:
            te=esp_timer_get_time();
            rate+=1;
            sample.type                 = BNO_TYPE_GAME_ROTATION;
            sample.game_rotation.i      = value->data.game_rotation_vector.i;
            sample.game_rotation.j      = value->data.game_rotation_vector.j;
            sample.game_rotation.k      = value->data.game_rotation_vector.k;
            sample.game_rotation.real   = value->data.game_rotation_vector.real;
            /*printf("(%.4f)%.4f,%.4f,%.4f,%.4f\n",
                    1000000/((te-ti)/rate),
                    value->data.game_rotation_vector.i, 
                    value->data.game_rotation_vector.j,
                    value->data.game_rotation_vector.k,
                    value->data.game_rotation_vector.real
                    );*/
            break;
        case BNO085_SENSOR_LINEAR_ACCELERATION:
            te=esp_timer_get_time();        
            rate+=1;
            sample.type                 = BNO_TYPE_LINEAR_ACCEL;
            sample.linear_accel.x       = value->data.linear_acceleration.x;
            sample.linear_accel.y       = value->data.linear_acceleration.y;
            sample.linear_accel.z       = value->data.linear_acceleration.z;
            /*printf("(%.4f)%.4f,%.4f,%.4f\n",
                    1000000/((te-ti)/rate),
                    value->data.linear_acceleration.x, 
                    value->data.linear_acceleration.y,
                    value-80>data.linear_acceleration.z
                    );*/
            break;

        default:
            break;
               
    }

    /*if ((rate % 194)==0) {
        ns+=1;
        printf("%.4f - %d\n",  (double)1000000/((te-ti)/rate), ns);
    }*/

    taskENTER_CRITICAL(&s_mux);
    uint8_t idx = s_active;
    log_sector_t *buf = &s_buf[idx];

    if (buf->header.sample_count < SAMPLES_PER_SECTOR) {
        buf->samples[buf->header.sample_count++] = sample;
    }
    if (buf->header.sample_count >= SAMPLES_PER_SECTOR) {
        uint8_t other = 1 - idx;
        if (s_buf[other].header.sample_count == 0) {
            // Other buffer already flushed -- safe to swap into it. 
            s_active = other;
            full_idx = idx;
        } else {
            // * Writer hasn't drained the other buffer yet. This means
            // * the flash writer is falling behind the 100 Hz sample
            // * rate (should not happen under normal conditions -- a
            // * 4 KB erase+write is on the order of tens of ms, versus
            // * the ~2 s it takes to fill a sector). We drop this
            // * sector's data rather than block the timer callback and
            //  * skew the sample cadence. 
            buf->header.sample_count = 0; // discard, keep sampling 
            s_stats.buffer_overruns++;
        }
    }
    taskEXIT_CRITICAL(&s_mux);

    if (full_idx != 0xFF) {
        BaseType_t ok = xQueueSend(s_flush_q, &full_idx, 0);
        if (ok != pdTRUE) {
            // Queue full (writer task starved) -- extremely unlikely
            // since it only ever holds at most one pending item in
            // this design, but handle it defensively. 
            taskENTER_CRITICAL(&s_mux);
            s_buf[full_idx].header.sample_count = 0;
            s_stats.buffer_overruns++;
            taskEXIT_CRITICAL(&s_mux);
        }
    }
    
}

esp_err_t init_imu() {

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
    
    

    return ESP_OK;
}

void start_imulogs() {
    // stop timesync
    espnow_time_initiator_stop();

    // flash log
    ESP_ERROR_CHECK(flash_log_start());

    if (IMU_ENABLE_LA) {
        bno085_enable_sensor(bno085, BNO085_SENSOR_LINEAR_ACCELERATION, IMU_LA_SAMPLING_RATE_HZ);
        ESP_LOGI(TAG, "IMU initialized, LA:%d hz", IMU_LA_SAMPLING_RATE_HZ );  
    }
    if (IMU_ENABLE_GRV) {
        bno085_enable_sensor(bno085, BNO085_SENSOR_GAME_ROTATION_VECTOR, IMU_GRV_SAMPLING_RATE_HZ);
        ESP_LOGI(TAG, "IMU initialized, GRV:%d hz", IMU_GRV_SAMPLING_RATE_HZ );  
    }

    // start imu logging
    gptimer_period=IMU_LA_SAMPLING_RATE_HZ;
    s_timesync_state=false;

    ti=esp_timer_get_time();
    te=rate=0;
}   

void stop_imulogs() {

    s_timesync_state=true;

    // disable IMU
    if (IMU_ENABLE_LA) {
        bno085_disable_sensor(bno085, BNO085_SENSOR_LINEAR_ACCELERATION);
        ESP_LOGI(TAG, "IMU LA disabled");  
    }
    if (IMU_ENABLE_GRV) {
        bno085_disable_sensor(bno085, BNO085_SENSOR_GAME_ROTATION_VECTOR);
        ESP_LOGI(TAG, "IMU GRV disabled");  
    }

    // stop flash logging
    ESP_ERROR_CHECK(flash_log_stop());

    // start espno   s_timesync_state=true;w timesync
    espnow_time_initiator_config_t config = {
        .sync_interval_ms = TIMESYNC_BROADCAST_INTERVAL_MS,  
    };
    espnow_time_initiator_start(&config);

    // start led blinking
    gptimer_period=TIMESYNC_BLINK_HZ;
}



// App main
void app_main()
{
    BaseType_t ok;

     // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Tasks
    s_gptimer_evt_q = xQueueCreate(4, sizeof(uint64_t));
    if (!s_gptimer_evt_q) {
        abort();
    }
    s_flush_q = xQueueCreate(2, sizeof(uint8_t));
    if (!s_flush_q) {
        abort();
    }
    ok = xTaskCreate(timer_task, "timer_task", 4096, NULL, 5, &s_gptimer_task);  
    if (ok != pdPASS) {
        abort();
    }
    ok = xTaskCreate(flash_task, "flash_task",  4096, NULL, tskIDLE_PRIORITY + 3, &s_writer_task);
    if (ok != pdPASS) {
        abort();
    }

    // SetUp
    ESP_ERROR_CHECK(init_battery());
    ESP_ERROR_CHECK(init_led());
    ESP_ERROR_CHECK(init_espnow_timesync());
    ESP_ERROR_CHECK(init_gptimer(esp_timer_get_time()));
    ESP_ERROR_CHECK(init_imu());
    ESP_ERROR_CHECK(init_flash());
    ESP_ERROR_CHECK(init_ble());

    ESP_LOGI(TAG, "Master ready - broadcasting every %d ms, blinking every %llu us",
             TIMESYNC_BROADCAST_INTERVAL_MS, gptimer_period);

    
}
