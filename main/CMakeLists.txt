idf_component_register(
    
    SRCS "main.c" "ble_control.c" "imu_flash_log.c"

    INCLUDE_DIRS "." 

    PRIV_REQUIRES   spi_flash 
                    nvs_flash
                    esp_timer
                    esp_driver_gptimer
                    esp_event 
                    esp_wifi 
                    esp_driver_gpio 
                    esp_driver_i2c  
                    bt
                    led_strip 
                    bno085
    )

    target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-address-of-packed-member")