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
        ESP_LOGI(MOTION_TAG, ">> Error: Motion Sensor not found - Check Hardware");
    }
    if (Motion.isConnected() == false) {
        ESP_LOGI(MOTION_TAG, ">> Error: Motion Sensor not found");
    }
    if (Motion.enableLinearAccelerometer() == true) {
        ESP_LOGI(MOTION_TAG, "Linear Accelerometer Activated  ");
        } else {
        ESP_LOGI(MOTION_TAG, "Linear Accelerometer Motion: Failed  ");
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
            ESP_LOGI(WIFI_TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(WIFI_TAG, "connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(WIFI_TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
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
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(WIFI_TAG, "wifi_init_sta finished.");

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
        ESP_LOGI(WIFI_TAG, "connected to ap SSID:%s password:%s",  CONFIG_ESP_WIFI_SSID, CONFIG_ESP_WIFI_PASSWORD);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(WIFI_TAG, "Failed to connect to SSID:%s, password:%s", CONFIG_ESP_WIFI_SSID, CONFIG_ESP_WIFI_PASSWORD);
    } else {
        ESP_LOGE(WIFI_TAG, "UNEXPECTED EVENT");
    }
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

void espnow_deinit(espnow_send_param_t *send_param)
{
    vQueueDelete(s_espnow_queue);
    s_espnow_queue = NULL;
    esp_now_deinit();
}

void espnow_init() {

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



// Led
static void led_task(void *p) {

    uint64_t meshUs = 0;
    bool flag=true;
    sensor_data_t incoming_data;
    const TickType_t xDelay = 5 / portTICK_PERIOD_MS;

    while (true) {
        meshUs = get_synced_time_us();
        uint32_t phase = (meshUs / 1000) % 10000;  // 0-999ms cycle
        bool ledOn = phase < 5000;
        if (ledOn) {
            if (flag) {
                ESP_LOGI(INFLUX_TAG, "ledOn: %" PRId64 "", get_synced_time_us());
                led_strip.LED(0,0,7);
                flag=!flag;
            }
            
        } else {
            if (!flag) {
                ESP_LOGI(INFLUX_TAG, "ledOff: %" PRId64 "", get_synced_time_us());
                led_strip.LED(0,0,0);
                flag=!flag;
            }
        }
        vTaskDelay(xDelay);
    }

}


void led_init() {

    led_strip.Init();
    xTaskCreate(led_task, "led_task", 2048, NULL, 4, NULL);
}




// InfluxDB
void write_to_influxdb(void *pvParameters) {
    sensor_data_t incoming_data;
    char post_data[128];

    while (1) {
        if (xQueueReceive(influx_queue, &incoming_data, portMAX_DELAY) == pdPASS) {

            snprintf(post_data, sizeof(post_data), "espnow,host=master drift=%" PRId32 ",timestamp=%" PRId64 "", incoming_data.drift, incoming_data.timestamp);

            esp_http_client_config_t config = {
                .url = INFLUX_URL,
                .method = HTTP_METHOD_POST,
                .timeout_ms = 5000,
            };

            esp_http_client_handle_t client = esp_http_client_init(&config);
            if (client == NULL) {
                    ESP_LOGE(INFLUX_TAG, "Failed to initialize HTTP client");
                    return;
            }

            esp_http_client_set_header(client, "Authorization", INFLUX_TOKEN);
            esp_http_client_set_header(client, "Content-Type", "text/plain; charset=utf-8");
            esp_http_client_set_header(client, "Accept", "application/json");

            esp_http_client_set_post_field(client, post_data, strlen(post_data));

            esp_err_t err = esp_http_client_perform(client);
            if (err == ESP_OK) {
                int status_code = esp_http_client_get_status_code(client);
                if (status_code == 204) {
                    ESP_LOGI(INFLUX_TAG, "Data successfully written to InfluxDB!");
                } else {
                    ESP_LOGE(INFLUX_TAG, "HTTP Post failed with status code: %d", status_code);
                }
            } else {
                ESP_LOGE(INFLUX_TAG, "HTTP POST request failed: %s", esp_err_to_name(err));
            }

            esp_http_client_cleanup(client);
        }
    }
}

void influxDBInit() {
    influx_queue = xQueueCreate(10, sizeof(sensor_data_t));
    if (influx_queue == NULL) {
        ESP_LOGE(INFLUX_TAG, "Failed to create InfluxDB queue");
        return;
    }

    influx_queue = xQueueCreate(5, sizeof(sensor_data_t));
    xTaskCreate(write_to_influxdb, "influx_task", 4096, NULL, 5, NULL);
}




// Main
void Initialize() {

    // init arduino libraries
    initArduino();

    // Serial init
    Serial.begin(115200);
    delay(1500);

    // Setup sensors enable pins
    SetupPins();

    // Battery init
    battery.Init();

    // WiFi init
    wifi_init();

    // Espnow init
    espnow_init();

    // Espnow time sync init
    espnow_timesync_init();

    // Init InfluxDB
    influxDBInit();

    // Init led
    led_init();

    // Init BNO085 motion reports
    //Motion_Init();


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