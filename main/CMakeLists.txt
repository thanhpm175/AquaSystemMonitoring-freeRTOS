idf_component_register(
    SRCS "main.cpp" "driver.cpp"
    INCLUDE_DIRS "."
    REQUIRES driver esp_wifi nvs_flash esp_netif esp_event
    PRIV_INCLUDE_DIRS "../lib/ssd1306/include"
)
