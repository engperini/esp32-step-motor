#include <stdbool.h>
#include <stdio.h>

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Edit these 3 pins to match your wiring on the XIAO ESP32-S3 Sense.
#ifndef STEP_GPIO
#define STEP_GPIO GPIO_NUM_4
#endif

#ifndef DIR_GPIO
#define DIR_GPIO GPIO_NUM_5
#endif

#ifndef EN_GPIO
#define EN_GPIO GPIO_NUM_6
#endif

// TMC2208 enable is active-low.
#ifndef EN_ACTIVE_LOW
#define EN_ACTIVE_LOW 1
#endif

// Pulse timing for the STEP input.
#ifndef STEP_PULSE_US
#define STEP_PULSE_US 5
#endif

#ifndef STEP_PERIOD_US
#define STEP_PERIOD_US 1000
#endif

#ifndef STEP_COUNT
#define STEP_COUNT 200
#endif

#ifndef STEP_DWELL_MS
#define STEP_DWELL_MS 1000
#endif

static const char *TAG = "step_motor";

static void motor_enable(bool enable)
{
    gpio_set_level(EN_GPIO, (enable && EN_ACTIVE_LOW) ? 0 : 1);
    if (!EN_ACTIVE_LOW) {
        gpio_set_level(EN_GPIO, enable ? 1 : 0);
    }
}

static void motor_step(bool dir, int steps)
{
    gpio_set_level(DIR_GPIO, dir ? 1 : 0);
    vTaskDelay(pdMS_TO_TICKS(2));

    for (int i = 0; i < steps; ++i) {
        gpio_set_level(STEP_GPIO, 1);
        esp_rom_delay_us(STEP_PULSE_US);
        gpio_set_level(STEP_GPIO, 0);
        esp_rom_delay_us(STEP_PERIOD_US);
    }
}

static void gpio_setup(void)
{
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << STEP_GPIO) | (1ULL << DIR_GPIO) | (1ULL << EN_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    ESP_ERROR_CHECK(gpio_config(&io));

    gpio_set_level(STEP_GPIO, 0);
    gpio_set_level(DIR_GPIO, 0);
    motor_enable(false);
}

void app_main(void)
{
    gpio_setup();

    ESP_LOGI(TAG, "STEP=%d DIR=%d EN=%d", STEP_GPIO, DIR_GPIO, EN_GPIO);
    ESP_LOGI(TAG, "Enable pin is %s", EN_ACTIVE_LOW ? "active-low" : "active-high");
    ESP_LOGI(TAG, "Pulse=%dus period=%dus steps=%d", STEP_PULSE_US, STEP_PERIOD_US, STEP_COUNT);

    motor_enable(true);
    ESP_LOGI(TAG, "Driver enabled. Starting test loop.");

    while (true) {
        ESP_LOGI(TAG, "Forward %d steps", STEP_COUNT);
        motor_step(true, STEP_COUNT);
        vTaskDelay(pdMS_TO_TICKS(STEP_DWELL_MS));

        ESP_LOGI(TAG, "Reverse %d steps", STEP_COUNT);
        motor_step(false, STEP_COUNT);
        vTaskDelay(pdMS_TO_TICKS(STEP_DWELL_MS));
    }
}
