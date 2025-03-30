#ifndef DRIVER_H
#define DRIVER_H

#include "config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/timers.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/adc.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "ArduinoJson.h"

#define TAG "DRIVER"

#define ServoMsMin 0.06
#define ServoMsMax 2.1
#define ServoMsAvg ((ServoMsMax - ServoMsMin) / 2.0)

// External variables
extern int servo_angle;

// Function declarations
void InitWiFi();
void initSensor();
void initServo();
void setServoAngle(int angle);
void handleServo();
void readSensors();
void processRPCRequest(const ArduinoJson::JsonVariantConst &params, ArduinoJson::JsonDocument &response);

// Task declarations
void wifi_task(void *pvParameters);
void thingsboard_task(void *pvParameters);
void sensor_task(void *pvParameters);
void servo_task(void *pvParameters);

// External variables
extern SensorData sensorData;
extern SystemState systemState;
extern TaskHandle_t wifi_task_handle;
extern TaskHandle_t thingsboard_task_handle;
extern TaskHandle_t sensor_task_handle;
extern TaskHandle_t servo_task_handle;
extern QueueHandle_t servo_queue;
extern QueueHandle_t sensor_queue;

// Add timer handle declaration
extern TimerHandle_t xServoTimer;

// Add timer callback declaration
void vServoTimerCallback(TimerHandle_t xTimer);

#endif // DRIVER_H