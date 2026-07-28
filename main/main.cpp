
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


// Espnow
void espnow

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


}

// App main
extern "C" void app_main()
{

    //Initialize NVS
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