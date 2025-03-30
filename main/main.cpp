#include "config.h"
#include "driver.h"

extern "C" void app_main(void)
{
    ESP_LOGI("MAIN", "[APP] Startup..");
    ESP_LOGI("MAIN", "[APP] Free memory: %" PRIu32 " bytes", esp_get_free_heap_size());
    ESP_LOGI("MAIN", "[APP] IDF version: %s", esp_get_idf_version());

    esp_log_level_set("*", ESP_LOG_INFO);

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // Initialize servo
    initServo();
    ESP_LOGI("MAIN", "Servo initialized");

    // Initialize OLED display
    init_oled();
    ESP_LOGI("MAIN", "OLED display initialized");

    // Create tasks with increased stack sizes
    xTaskCreate(wifi_task, "wifi_task", 4096, NULL, 5, &wifi_task_handle);
    xTaskCreate(thingsboard_task, "thingsboard_task", 8192, NULL, 4, &thingsboard_task_handle);
    xTaskCreate(sensor_task, "sensor_task", 4096, NULL, 3, &sensor_task_handle);
    xTaskCreate(servo_task, "servo_task", 4096, NULL, 2, &servo_task_handle);
    xTaskCreate(oled_task, "oled_task", 4096, NULL, 1, &oled_task_handle);

    // Main loop - just keep the system running
    while (1)
    {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}