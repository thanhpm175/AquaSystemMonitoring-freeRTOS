#include "driver.h"
#include "ThingsBoard.h"
#include "Server_Side_RPC.h"
#include "Shared_Attribute_Update.h"
#include "Espressif_MQTT_Client.h"
#include "driver/adc.h"
#include "ssd1306.h"

// Initialize global variables
SensorData sensorData = {0.0f, 0.0f, 0.0f, 0.0f};
SystemState systemState = {false, false, false, false, 0};
int servo_angle = 0;

// Task handles
TaskHandle_t wifi_task_handle = NULL;
TaskHandle_t thingsboard_task_handle = NULL;
TaskHandle_t sensor_task_handle = NULL;
TaskHandle_t servo_task_handle = NULL;

// Queue handles
QueueHandle_t servo_queue = NULL;
QueueHandle_t sensor_queue = NULL;

// Initialize ThingsBoard components
Espressif_MQTT_Client<> mqttClient;
Shared_Attribute_Update<1U, MAX_ATTRIBUTES> shared_update;
Server_Side_RPC<4U, 4U> rpc;
const std::array<IAPI_Implementation *, 2U> apis = {
    &shared_update,
    &rpc};
ThingsBoardSized<1024> tb(mqttClient, 1024, 1024, TB_STACK_SIZE, apis);

// Add timer handle definition
TimerHandle_t xServoTimer;

// Add OLED handle definition
SSD1306_t oled_dev;
TaskHandle_t oled_task_handle = NULL;

// Initialize previous values
PreviousValues prev_values = {
    .temperature = 0.0f,
    .turbidity = 0.0f,
    .ph = 0.0f,
    .tds = 0.0f,
    .servo_active = false};

// WiFi Functions Implementation
void on_got_ip(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    systemState.wifi_connected = true;
}

void InitWiFi()
{
    ESP_LOGI("MAIN", "Initializing WiFi...");
    const wifi_init_config_t wifi_init_config = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_init_config));

    esp_netif_config_t netif_config = ESP_NETIF_DEFAULT_WIFI_STA();
    esp_netif_t *netif = esp_netif_new(&netif_config);
    assert(netif);

    ESP_ERROR_CHECK(esp_netif_attach_wifi_station(netif));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, ip_event_t::IP_EVENT_STA_GOT_IP, &on_got_ip, NULL));
    ESP_ERROR_CHECK(esp_wifi_set_default_wifi_sta_handlers());
    ESP_ERROR_CHECK(esp_wifi_set_storage(wifi_storage_t::WIFI_STORAGE_RAM));

    wifi_config_t wifi_config;
    memset(&wifi_config, 0, sizeof(wifi_config));
    strncpy(reinterpret_cast<char *>(wifi_config.sta.ssid), WIFI_SSID, strlen(WIFI_SSID) + 1);
    strncpy(reinterpret_cast<char *>(wifi_config.sta.password), WIFI_PASSWORD, strlen(WIFI_PASSWORD) + 1);

    ESP_LOGI("MAIN", "Connecting to WiFi SSID: %s", wifi_config.sta.ssid);
    ESP_ERROR_CHECK(esp_wifi_set_mode(wifi_mode_t::WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(wifi_interface_t::WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_connect());
    ESP_LOGI("MAIN", "WiFi initialization completed");
}

void initSensor()
{
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(PH_SENSOR_GPIO, ADC_ATTEN_DB_12);
    adc1_config_channel_atten(TURBIDITY_SENSOR_GPIO, ADC_ATTEN_DB_12);
}

// Servo Functions Implementation
void initServo()
{
    // Configure LEDC timer
    ledc_timer_config_t ledc_timer = {
        .speed_mode = SERVO_MODE,
        .duty_resolution = SERVO_RESOLUTION,
        .timer_num = SERVO_TIMER,
        .freq_hz = SERVO_FREQ,
        .clk_cfg = LEDC_AUTO_CLK,
        .deconfigure = false};
    ledc_timer_config(&ledc_timer);

    // Configure LEDC channel
    ledc_channel_config_t ledc_channel = {
        .gpio_num = SERVO_GPIO,
        .speed_mode = SERVO_MODE,
        .channel = SERVO_CHANNEL,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = SERVO_TIMER,
        .duty = SERVO_DUTY_MIN,
        .hpoint = 0,
        .flags = {
            .output_invert = 0}};
    ledc_channel_config(&ledc_channel);

    // Create timer for servo return
    xServoTimer = xTimerCreate(
        "ServoTimer",
        pdMS_TO_TICKS(5000),
        pdFALSE,
        NULL,
        vServoTimerCallback);

    if (xServoTimer == NULL)
    {
        ESP_LOGE(TAG, "Failed to create servo timer");
    }
}

void setServoAngle(int angle)
{
    if (angle == 1)
        angle = ServoMsAvg;
    if (angle == 0)
        angle = ServoMsMax;

    int duty = (int)(100.0 * (angle / 20.0) * 81.91);

    ESP_LOGI(TAG, "Setting servo angle to: %d, duty cycle: %d", angle, duty);

    // Use mutex to protect servo control
    static SemaphoreHandle_t servo_mutex = NULL;
    if (servo_mutex == NULL)
    {
        servo_mutex = xSemaphoreCreateMutex();
    }
    xSemaphoreTake(servo_mutex, portMAX_DELAY);

    ledc_set_duty(SERVO_MODE, SERVO_CHANNEL, duty);
    ledc_update_duty(SERVO_MODE, SERVO_CHANNEL);

    xSemaphoreGive(servo_mutex);
}

void handleServo()
{
    static uint32_t last_update = 0;
    uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;

    // Only update servo if it's active or if we need to return to original position
    if (systemState.servo_active || systemState.servo_moving)
    {
        if (!systemState.servo_moving)
        {
            systemState.servo_start_time = current_time;
            systemState.servo_moving = true;
            setServoAngle(1); // Set to 90 degrees when active
            ESP_LOGI(TAG, "Servo activated, moving to 90 degrees");
        }
        else if (current_time - systemState.servo_start_time >= 5000)
        {
            setServoAngle(0);
            systemState.servo_moving = false;
            systemState.servo_active = false;
            ESP_LOGI(TAG, "Servo deactivated, returned to 0 degrees");
        }
    }
}

float read_temp_sensor()
{
    // TODO: Implement temperature sensor reading
    return 26.0f;
}

float read_ph_sensor()
{
    int raw = adc1_get_raw(PH_SENSOR_GPIO);
    float voltage = raw * (3.3f / 4095.0f);
    float ph = 20.5940f - (5.4450f * voltage);
    return ph;
}

float read_turbidity_sensor()
{
    float sum_voltage = 0;
    for (int i = 0; i < 800; i++)
    {
        int raw = adc1_get_raw(TURBIDITY_SENSOR_GPIO);
        sum_voltage += raw * (3.3f / 4095.0f);
    }
    float voltage = sum_voltage / 800.0f;

    float NTU = 0;
    if (voltage < 0.36f)
    {
        NTU = 3000.0f;
    }
    else if (voltage > 1.8f)
    {
        NTU = 0.0f;
    }
    else
    {
        NTU = (-1120.4f * (voltage + 2.4f) * (voltage + 2.4f)) +
              (5742.3f * (voltage + 2.4f)) - 4352.9f;
    }
    return NTU;
}

float read_tds_sensor()
{
    // TODO: Implement TDS sensor reading
    return 100.0f;
}

void processRPCRequest(const JsonVariantConst &params, JsonDocument &response)
{
    ESP_LOGI(TAG, "Received RPC request");

    // Log the received params for debugging
    char params_str[256];
    serializeJson(params, params_str);
    ESP_LOGI(TAG, "Received params: %s", params_str);

    // Check for method field
    if (!params.containsKey("method"))
    {
        ESP_LOGE(TAG, "Missing method parameter");
        response["error"] = "Missing method parameter";
        return;
    }

    const char *method = params["method"];
    ESP_LOGI(TAG, "Method: %s", method);

    if (strcmp(method, "setServoState") == 0)
    {
        // Check for params object and angle parameter
        if (!params.containsKey("params") || !params["params"].containsKey("angle"))
        {
            ESP_LOGE(TAG, "Missing angle parameter");
            response["error"] = "Missing angle parameter";
            return;
        }

        int new_angle = params["params"]["angle"];
        if (new_angle < SERVO_ANGLE_MIN || new_angle > SERVO_ANGLE_MAX)
        {
            ESP_LOGE(TAG, "Invalid angle: %d", new_angle);
            response["error"] = "Invalid angle (0-180)";
            return;
        }

        ESP_LOGI(TAG, "Setting servo angle to: %d", new_angle);
        servo_angle = new_angle;

        // Set servo state and start timer in a safe way
        systemState.servo_active = true;
        systemState.servo_moving = false;
        systemState.servo_start_time = xTaskGetTickCount() * portTICK_PERIOD_MS;

        setServoAngle(new_angle);

        response["success"] = true;
        response["angle"] = new_angle;
        response["message"] = "Servo angle set successfully";
    }
    else
    {
        ESP_LOGE(TAG, "Unknown method: %s", method);
        response["error"] = "Unknown method";
    }
}

// Task Functions Implementation
void wifi_task(void *pvParameters)
{
    ESP_LOGI("WIFI", "Starting WiFi task");
    InitWiFi();

    while (1)
    {
        if (!systemState.wifi_connected)
        {
            ESP_LOGI("WIFI", "Waiting for WiFi connection...");
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            continue;
        }
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}

void thingsboard_task(void *pvParameters)
{
    ESP_LOGI("TB", "Starting ThingsBoard task");

    while (1)
    {
        if (!systemState.wifi_connected)
        {
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            continue;
        }

        if (!tb.connected())
        {
            ESP_LOGI("TB", "Connecting to ThingsBoard server: %s with token: %s",
                     THINGSBOARD_SERVER, TOKEN);
            if (!tb.connect(THINGSBOARD_SERVER, TOKEN, THINGSBOARD_PORT))
            {
                ESP_LOGE("TB", "Failed to connect to ThingsBoard server");
                vTaskDelay(5000 / portTICK_PERIOD_MS);
                continue;
            }
            ESP_LOGI("TB", "Connected to ThingsBoard server successfully");

            // Register RPC callback
            RPC_Callback callback(SERVO_CONTROL_RPC, processRPCRequest);
            if (rpc.RPC_Subscribe(callback))
            {
                ESP_LOGI("TB", "Registered RPC handler for: %s", SERVO_CONTROL_RPC);
            }
            else
            {
                ESP_LOGE("TB", "Failed to register RPC handler for: %s", SERVO_CONTROL_RPC);
            }
        }

        tb.loop();

        static uint32_t last_update = 0;
        uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
        if (current_time - last_update > 10000)
        {
            if (tb.sendTelemetryData(SERVO_STATE_KEY, systemState.servo_active))
            {
                ESP_LOGI("TB", "Sent servo state: %d", systemState.servo_active);
            }
            else
            {
                ESP_LOGE("TB", "Failed to send servo state");
            }
            last_update = current_time;
        }

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void sensor_task(void *pvParameters)
{
    ESP_LOGI("SENSOR", "Starting sensor task");

    // Wait for WiFi to be connected before initializing sensor
    while (!systemState.wifi_connected)
    {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    // Initialize sensor after WiFi is connected
    initSensor();
    ESP_LOGI("SENSOR", "Sensor initialized");

    while (1)
    {
        static uint32_t last_read = 0;
        uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;

        // Read sensors every 10 seconds
        if (current_time - last_read > 10000)
        {
            sensorData.temperature = read_temp_sensor();
            sensorData.turbidity = read_turbidity_sensor();
            sensorData.ph = read_ph_sensor();
            sensorData.tds = read_tds_sensor();
            last_read = current_time;
        }

        if (tb.connected())
        {
            static uint32_t last_update = 0;
            if (current_time - last_update > 10000)
            {
                ESP_LOGI("SENSOR", "Sending sensor data - Temperature: %.2f, Turbidity: %.2f, PH: %.2f, TDS: %.2f",
                         sensorData.temperature, sensorData.turbidity, sensorData.ph, sensorData.tds);

                if (!tb.sendTelemetryData(TEMPERATURE_KEY, sensorData.temperature))
                {
                    ESP_LOGE("SENSOR", "Failed to send temperature data");
                }
                if (!tb.sendTelemetryData(TURBIDITY_KEY, sensorData.turbidity))
                {
                    ESP_LOGE("SENSOR", "Failed to send turbidity data");
                }
                if (!tb.sendTelemetryData(PH_KEY, sensorData.ph))
                {
                    ESP_LOGE("SENSOR", "Failed to send pH data");
                }
                if (!tb.sendTelemetryData(TDS_KEY, sensorData.tds))
                {
                    ESP_LOGE("SENSOR", "Failed to send TDS data");
                }

                last_update = current_time;
            }
        }

        vTaskDelay(1000 / portTICK_PERIOD_MS); // Check every second
    }
}

void servo_task(void *pvParameters)
{
    ESP_LOGI("SERVO", "Starting servo task");

    while (1)
    {
        handleServo();

        if (tb.connected())
        {
            static uint32_t last_update = 0;
            uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
            if (current_time - last_update > 5000)
            {
                if (tb.sendTelemetryData(SERVO_STATE_KEY, systemState.servo_active))
                {
                    ESP_LOGI("SERVO", "Sent servo state: %d", systemState.servo_active);
                }
                last_update = current_time;
            }
        }

        vTaskDelay(1000 / portTICK_PERIOD_MS); // Increased delay to 500ms since we don't need frequent updates
    }
}

// Add timer callback function
void vServoTimerCallback(TimerHandle_t xTimer)
{
    // Return to original position using the common setServoAngle function
    setServoAngle(0);
    ESP_LOGI(TAG, "Servo returned to original position");
}

void init_oled(void)
{
    i2c_master_init(&oled_dev, OLED_SDA_GPIO, OLED_SCL_GPIO, -1);
    i2c_device_add(&oled_dev, I2C_NUM_0, -1, OLED_I2C_ADDRESS);
    i2c_init(&oled_dev, 128, 64);
    ssd1306_clear_screen(&oled_dev, false);
}

void update_oled_display(void)
{
    // Check if any values have changed
    bool values_changed =
        (sensorData.temperature != prev_values.temperature) ||
        (sensorData.turbidity != prev_values.turbidity) ||
        (sensorData.ph != prev_values.ph) ||
        (sensorData.tds != prev_values.tds) ||
        (systemState.servo_active != prev_values.servo_active);

    // Only update display if values have changed
    if (values_changed)
    {
        ssd1306_clear_screen(&oled_dev, false);

        // Display title
        const char *title = "Aqua Monitor";
        ssd1306_display_text(&oled_dev, 0, (char *)title, 12, false);

        // Display sensor values
        char temp_str[32];
        char ph_str[32];
        char turb_str[32];
        char tds_str[32];

        snprintf(temp_str, sizeof(temp_str), "Temp: %.1f C", sensorData.temperature);
        snprintf(ph_str, sizeof(ph_str), "pH: %.1f", sensorData.ph);
        snprintf(turb_str, sizeof(turb_str), "Turb: %.0f NTU", sensorData.turbidity);
        snprintf(tds_str, sizeof(tds_str), "TDS: %.0f ppm", sensorData.tds);

        ssd1306_display_text(&oled_dev, 2, temp_str, strlen(temp_str), false);
        ssd1306_display_text(&oled_dev, 3, ph_str, strlen(ph_str), false);
        ssd1306_display_text(&oled_dev, 4, turb_str, strlen(turb_str), false);
        ssd1306_display_text(&oled_dev, 5, tds_str, strlen(tds_str), false);

        // Display servo status
        char servo_str[32];
        snprintf(servo_str, sizeof(servo_str), "Servo: %s", systemState.servo_active ? "ON" : "OFF");
        ssd1306_display_text(&oled_dev, 7, servo_str, strlen(servo_str), false);

        // Update previous values
        prev_values.temperature = sensorData.temperature;
        prev_values.turbidity = sensorData.turbidity;
        prev_values.ph = sensorData.ph;
        prev_values.tds = sensorData.tds;
        prev_values.servo_active = systemState.servo_active;
    }
}

void oled_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Starting OLED task");

    while (1)
    {
        update_oled_display();
        vTaskDelay(pdMS_TO_TICKS(1000)); // Check every second
    }
}