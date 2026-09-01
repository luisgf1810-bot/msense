idf_component_register(
    
    SRCS "main.c" "imu_flash_log.c" "ble_control.c" 

    INCLUDE_DIRS "." 

    PRIV_REQUIRES   spi_flash 
                    nvs_flash
                    esp_rom 
                    esp_hw_support
                    esp_timer
                    esp_driver_gptimer
                    esp_event 
                    esp_wifi
                    esp_netif
                    esp_driver_gpio 
                    esp_driver_i2c  
                    esp_partition
                    bt
                    led_strip 
                    bno085
    )

    target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-address-of-packed-member")