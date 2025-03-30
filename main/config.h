#ifndef CONFIG_H
#define CONFIG_H

#include <esp_netif.h>
#include <esp_log.h>
#include <esp_wifi.h>
#include <nvs_flash.h>
#include <string.h>
#include <driver/ledc.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include <stdint.h>

// WiFi Configuration
#define WIFI_SSID "NHATRO BM T1S"
#define WIFI_PASSWORD "nhatro123456t1"

// ThingsBoard Configuration
#define THINGSBOARD_SERVER "192.168.110.126"
#define TOKEN "koPUeob0V2iGJO646g2B"
#define THINGSBOARD_PORT 1883

// GPIO Configuration
#define SERVO_GPIO 13
#define PH_SENSOR_GPIO ADC1_CHANNEL_0
#define TURBIDITY_SENSOR_GPIO ADC1_CHANNEL_1

// Servo Configuration
#define SERVO_MODE LEDC_LOW_SPEED_MODE
#define SERVO_CHANNEL LEDC_CHANNEL_0
#define SERVO_TIMER LEDC_TIMER_0
#define SERVO_FREQ 50
#define SERVO_RESOLUTION LEDC_TIMER_13_BIT
#define SERVO_DUTY_MIN 0
#define SERVO_DUTY_MAX 8191
#define SERVO_ANGLE_MIN 0
#define SERVO_ANGLE_MAX 180

#define ServoMsMin 0.06
#define ServoMsMax 2.1
#define ServoMsAvg ((ServoMsMax - ServoMsMin) / 2.0)

// Task Configuration
#define MAX_MESSAGE_RECEIVE_SIZE 1024
#define MAX_MESSAGE_SEND_SIZE 1024
#define TB_STACK_SIZE 4096
#define MAX_ATTRIBUTES 10

// Telemetry Keys
#define TEMPERATURE_KEY "temperature"
#define TURBIDITY_KEY "turbidity"
#define PH_KEY "ph"
#define TDS_KEY "tds"
#define SERVO_STATE_KEY "servo_active"

// RPC Configuration
#define SERVO_CONTROL_RPC "setServoState"

// Sensor Data Structure
struct SensorData
{
    float temperature;
    float turbidity;
    float ph;
    float tds;
};

// System State Structure
struct SystemState
{
    bool wifi_connected;
    bool subscribed;
    bool servo_active;
    bool servo_moving;
    uint32_t servo_start_time;
};

// Task Handles
extern TaskHandle_t wifi_task_handle;
extern TaskHandle_t thingsboard_task_handle;
extern TaskHandle_t sensor_task_handle;
extern TaskHandle_t servo_task_handle;

// Queue Handles
extern QueueHandle_t servo_queue;
extern QueueHandle_t sensor_queue;

#endif // CONFIG_H