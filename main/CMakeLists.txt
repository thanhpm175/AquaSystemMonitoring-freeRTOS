idf_component_register(
    SRCS "main.cpp" "driver.cpp"
    INCLUDE_DIRS "."
    REQUIRES driver esp_wifi nvs_flash esp_netif esp_event ssd1306
    PRIV_INCLUDE_DIRS "../components/ssd1306/include"
)
