#include <stdbool.h>
#include <stdint.h>
#include <inttypes.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Edit these pins to match your wiring on the XIAO ESP32-S3 Sense.
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

// STEP frequency target in microseconds per pulse period.
#ifndef STEP_PERIOD_US
#define STEP_PERIOD_US 1000
#endif

// Time-based movement test.
#ifndef MOVE_TIME_MS
#define MOVE_TIME_MS 5000
#endif

#ifndef PAUSE_MS
#define PAUSE_MS 1000
#endif

// A short direction setup time before starting step pulses.
#ifndef DIR_SETUP_US
#define DIR_SETUP_US 20
#endif

static const char *TAG = "step_motor";

static const ledc_mode_t LEDC_SPEED_MODE = LEDC_LOW_SPEED_MODE;
static const ledc_timer_t LEDC_TIMER_NUM = LEDC_TIMER_0;
static const ledc_channel_t LEDC_CHANNEL_NUM = LEDC_CHANNEL_0;

static uint32_t step_frequency_hz(void)
{
    return (STEP_PERIOD_US == 0U) ? 0U : (1000000U / STEP_PERIOD_US);
}

static uint32_t ledc_resolution_bits_for_frequency(uint32_t freq_hz)
{
    // Keep enough resolution for a clean 50% duty cycle while staying valid
    // for the requested frequency. Lower resolution is acceptable if needed.
    if (freq_hz <= 1000U) {
        return LEDC_TIMER_10_BIT;
    }

    if (freq_hz <= 5000U) {
        return LEDC_TIMER_9_BIT;
    }

    return LEDC_TIMER_8_BIT;
}

static void motor_enable(bool enable)
{
    if (EN_ACTIVE_LOW) {
        gpio_set_level(EN_GPIO, enable ? 0 : 1);
    } else {
        gpio_set_level(EN_GPIO, enable ? 1 : 0);
    }
}

static void step_output_start(void)
{
    const uint32_t max_duty = (1U << ledc_resolution_bits_for_frequency(step_frequency_hz())) - 1U;
    const uint32_t duty = max_duty / 2U;

    ESP_ERROR_CHECK(ledc_set_duty(LEDC_SPEED_MODE, LEDC_CHANNEL_NUM, duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_SPEED_MODE, LEDC_CHANNEL_NUM));
}

static void step_output_stop(void)
{
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_SPEED_MODE, LEDC_CHANNEL_NUM, 0));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_SPEED_MODE, LEDC_CHANNEL_NUM));
}

static void motor_run_for_ms(bool dir, uint32_t duration_ms)
{
    gpio_set_level(DIR_GPIO, dir ? 1 : 0);
    esp_rom_delay_us(DIR_SETUP_US);

    step_output_start();
    vTaskDelay(pdMS_TO_TICKS(duration_ms));
    step_output_stop();
}

static void motor_setup(void)
{
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << DIR_GPIO) | (1ULL << EN_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    ESP_ERROR_CHECK(gpio_config(&io));

    gpio_set_level(DIR_GPIO, 0);
    motor_enable(false);

    const uint32_t freq_hz = step_frequency_hz();
    const uint32_t resolution_bits = ledc_resolution_bits_for_frequency(freq_hz);

    ledc_timer_config_t timer = {
        .speed_mode = LEDC_SPEED_MODE,
        .duty_resolution = resolution_bits,
        .timer_num = LEDC_TIMER_NUM,
        .freq_hz = freq_hz,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));

    ledc_channel_config_t channel = {
        .gpio_num = STEP_GPIO,
        .speed_mode = LEDC_SPEED_MODE,
        .channel = LEDC_CHANNEL_NUM,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_NUM,
        .duty = 0,
        .hpoint = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&channel));

    ESP_LOGI(TAG, "STEP=%d DIR=%d EN=%d", STEP_GPIO, DIR_GPIO, EN_GPIO);
    ESP_LOGI(TAG, "Enable pin is %s", EN_ACTIVE_LOW ? "active-low" : "active-high");
    ESP_LOGI(TAG, "PWM step frequency=%" PRIu32 " Hz (%" PRIu32 " us period), resolution=%" PRIu32 " bits", freq_hz, (uint32_t)STEP_PERIOD_US, resolution_bits);
    ESP_LOGI(TAG, "Move=%" PRIu32 " ms pause=%" PRIu32 " ms", (uint32_t)MOVE_TIME_MS, (uint32_t)PAUSE_MS);
}

void app_main(void)
{
    motor_setup();

    motor_enable(true);
    ESP_LOGI(TAG, "Driver enabled. Starting hardware-PWM direction test.");

    while (true) {
        ESP_LOGI(TAG, "Forward for %u ms", MOVE_TIME_MS);
        motor_run_for_ms(true, MOVE_TIME_MS);
        vTaskDelay(pdMS_TO_TICKS(PAUSE_MS));

        ESP_LOGI(TAG, "Reverse for %u ms", MOVE_TIME_MS);
        motor_run_for_ms(false, MOVE_TIME_MS);
        vTaskDelay(pdMS_TO_TICKS(PAUSE_MS));
    }
}
