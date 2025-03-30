idf_component_register(SRCS "main.cpp" "driver.cpp"
                    INCLUDE_DIRS "."
                    "../lib/ssd1306/include"
                    REQUIRES driver esp_wifi nvs_flash esp_netif esp_event )
