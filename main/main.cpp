#include "main.hpp"



// Setup functions
void SetupPins() {

    pinMode(LED_ON_PIN, OUTPUT);    /*Set LED transistor pin as output*/
    digitalWrite(LED_ON_PIN, HIGH); /*Init Set up to output high*/
    pinMode(SENS_ON_PIN, OUTPUT); /*Set LDO enable pin as output*/
    gpio_hold_dis((gpio_num_t)SENS_ON_PIN);
    digitalWrite(SENS_ON_PIN, HIGH); /*Init Set up to output high*/
    pinMode(LIGHT_WAKEUP_PIN, INPUT);   // Assumes active-low button
    pinMode(MOTION_WAKEUP_PIN, INPUT);  // Assumes active-low button

}


// Motion sensor functions
void Motion_Init() {

    Wire.begin(8, 9, 400000);  // SDA on GPIO8, SCL on GPIO9, 400kHz speed
    if (Motion.begin() == false) {
        ESP_LOGI(MAIN, ">> Error: Motion Sensor not found - Check Hardware");
    }
    if (Motion.isConnected() == false) {
        ESP_LOGI(MAIN, ">> Error: Motion Sensor not found");
    }
    if (Motion.enableLinearAccelerometer() == true) {
        ESP_LOGI(MAIN, "Linear Accelerometer Activated  ");
        } else {
        ESP_LOGI(MAIN, "Linear Accelerometer Motion: Failed  ");
    }

    _i2c_write_size = 0;
}

void Motion_Read() {
    bool error_flag = 1;
    uint8_t motion_id = 0xFF;
 
    while (Motion.getSensorEvent() == true) {
        motion_id = Motion.getSensorEventID();
        //ESP_LOGI(MAIN, "Sensror Event ID: %i", motion_id);

        if (motion_id == SENSOR_REPORTID_LINEAR_ACCELERATION) {
                _motion_data[20] = Motion.getLinAccelX();
                _motion_data[21] = Motion.getLinAccelY();
                _motion_data[22] = Motion.getLinAccelZ();
                error_flag = 0;
                break;
        }

        if (error_flag) {
            ESP_LOGI(MOTION_TAG, ">> Error: Motion Sensor not found");
        }
    }
}

void motion_task(void *pvParameter) {
    LedStrip led_strip;

    led_strip.Init();

    while(1) {
        led_strip.LED(0, LED_DEFAULT_BRIGHTNESS, 0);

        start_time = esp_timer_get_time();
        Motion_Read();
        end_time = esp_timer_get_time();
        //ESP_LOGI(MOTION_TAG, "Acc XYZ: %.2f, %.2f, %.2f m/s^2 ", _motion_data[20], _motion_data[21], _motion_data[22]);
        //ESP_LOGI(MOTION_TAG, "Motion read time: %lldus", end_time - start_time);
        led_strip.LED(0, 0, 0);
        vTaskDelay(1000 / portTICK_PERIOD_MS); // Wait 1 second
    }
}


// Wifi 
static void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < CONFIG_ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(MAIN, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(MAIN,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(MAIN, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static void wifi_init()
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = CONFIG_ESP_WIFI_SSID,
            .password = CONFIG_ESP_WIFI_PASSWORD,
            /* Authmode threshold resets to WPA2 as default if auth mode threshold equals WIFI_AUTH_OPEN
             * and password matches WPA2 standards (password len => 8).
             * If you want to connect the device to deprecated WEP/WPA networks, Please set the threshold value
             * to WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK and set the password with length and format matching to
             * WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK standards.
             */
            .sae_pwe_h2e = ESP_WIFI_SAE_MODE,
            .sae_h2e_identifier = H2E_IDENTIFIER,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(MAIN, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(MAIN, "connected to ap SSID:%s password:%s",  CONFIG_ESP_WIFI_SSID, CONFIG_ESP_WIFI_PASSWORD);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(MAIN, "Failed to connect to SSID:%s, password:%s", CONFIG_ESP_WIFI_SSID, CONFIG_ESP_WIFI_PASSWORD);
    } else {
        ESP_LOGE(MAIN, "UNEXPECTED EVENT");
    }
}


// ESPNOW time sync
static void timesync_event_handler(void *arg, esp_event_base_t base, int32_t event_id, void *event_data)
{
    if (event_id == ESP_EVENT_ESPNOW_TIMESYNC_SYNCED) {
        espnow_timesync_event_t *evt = (espnow_timesync_event_t *)event_data;
        s_time_offset_us = evt->synced_time_us - esp_timer_get_time();
        ESP_LOGI(MAIN, "Time synced from " MACSTR ", drift: %" PRId32 " ms", MAC2STR(evt->src_addr), evt->drift_ms);
    }
}

// Get current synchronized time
static int64_t get_synced_time_us(void)
{
    return esp_timer_get_time() + s_time_offset_us;
}

// Espnow

int espnow_data_parse(uint8_t *data, uint16_t data_len, uint8_t *state, uint16_t *seq, uint32_t *magic)
{
    espnow_data_t *buf = (espnow_data_t *)data;
    uint16_t crc, crc_cal = 0;

    if (data_len < sizeof(espnow_data_t)) {
        ESP_LOGE(MAIN, "Receive ESPNOW data too short, len:%d", data_len);
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
        ESP_LOGE(MAIN, "Receive cb arg error");
        return;
    }

    evt.id = ESPNOW_RECV_CB;
    memcpy(recv_cb->mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
    recv_cb->data = (uint8_t *)malloc(len);
    if (recv_cb->data == NULL) {
        ESP_LOGE(MAIN, "Malloc receive data fail");
        return;
    }
    memcpy(recv_cb->data, data, len);
    recv_cb->data_len = len;
    if (xQueueSend(s_espnow_queue, &evt, ESPNOW_MAXDELAY) != pdTRUE) {
        ESP_LOGW(MAIN, "Send receive queue fail");
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
                    ESP_LOGI(MAIN, "Receive %dth broadcast data from: "MACSTR"", recv_seq, MAC2STR(recv_cb->mac_addr), recv_cb->data_len);

                }
                else if (ret == ESPNOW_DATA_UNICAST) {
                    ESP_LOGI(MAIN, "Receive %dth unicast data from: "MACSTR", len: %d", recv_seq, MAC2STR(recv_cb->mac_addr), recv_cb->data_len);

                }
                else {
                    ESP_LOGI(MAIN, "Receive error data from: "MACSTR"", MAC2STR(recv_cb->mac_addr));
                }
                break;
            }
            default:
                ESP_LOGE(MAIN, "Callback type error: %d", evt.id);
                break;
        }
    }
}

void espnow_deinit()
{
    vQueueDelete(s_espnow_queue);
    s_espnow_queue = NULL;
    esp_now_deinit();
}

void espnow_init() {

    s_espnow_queue = xQueueCreate(ESPNOW_QUEUE_SIZE, sizeof(espnow_event_t));
    if (s_espnow_queue == NULL) {
        ESP_LOGE(MAIN, "Create queue fail");
        esp_now_deinit();
        return;
    }

    ESP_ERROR_CHECK( esp_now_init() );
    ESP_ERROR_CHECK( esp_now_register_recv_cb(espnow_recv_cb) );
    ESP_ERROR_CHECK( esp_now_set_pmk((const uint8_t *)CONFIG_ESPNOW_PMK) );

    xTaskCreate(espnow_task, "espnow_task", 2048, NULL, 4, NULL);
}





// Main
void Initialize() {

    // init arduino libraries
    initArduino();

    // Setup sensors enable pins
    SetupPins();

    // Serial init
    Serial.begin(115200);
    delay(1500);

    // Battery init
    battery.Init();

    // Init BNO085 motion reports
    //Motion_Init();
    
    // Init led
    led_strip.Init();

    // WiFi init
    wifi_init();

    // Espnow init
    espnow_init();

}






// App main
extern "C" void app_main()
{

    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Init components
    Initialize();


    //ESP_LOGI(TAG, "Battery voltage read: %i", battery.BatteryVoltageRead());

    // WARNING: if program reaches end of function app_main() the MCU will restart.
}