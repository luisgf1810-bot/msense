idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "include" "../components/bno085/include" "../components/Battery/include" "../components/LedStrip/include" "../components/espnowdev/include"
    PRIV_REQUIRES nvs_flash esp_event esp_wifi
    REQUIRES bno085 Battery LedStrip )