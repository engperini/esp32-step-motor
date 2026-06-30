#include <ctype.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_app_desc.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "lwip/inet.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "index_html.h"

#ifndef MOTOR_STEPS_PER_REV
#define MOTOR_STEPS_PER_REV 200U
#endif

#ifndef MOTOR_MAX_RPM
#define MOTOR_MAX_RPM 300U
#endif

#ifndef MOTOR1_STEP_GPIO
#define MOTOR1_STEP_GPIO GPIO_NUM_4
#endif

#ifndef MOTOR1_DIR_GPIO
#define MOTOR1_DIR_GPIO GPIO_NUM_5
#endif

#ifndef MOTOR1_EN_GPIO
#define MOTOR1_EN_GPIO GPIO_NUM_6
#endif

#ifndef MOTOR2_STEP_GPIO
#define MOTOR2_STEP_GPIO GPIO_NUM_7
#endif

#ifndef MOTOR2_DIR_GPIO
#define MOTOR2_DIR_GPIO GPIO_NUM_8
#endif

#ifndef MOTOR2_EN_GPIO
#define MOTOR2_EN_GPIO GPIO_NUM_9
#endif

#ifndef EN_ACTIVE_LOW
#define EN_ACTIVE_LOW 1
#endif

#ifndef STEP_PERIOD_US
#define STEP_PERIOD_US 1000U
#endif

#ifndef MOVE_TIME_MS
#define MOVE_TIME_MS 5000U
#endif

#ifndef STEP_COUNT
#define STEP_COUNT 200U
#endif

#ifndef PAUSE_MS
#define PAUSE_MS 1000U
#endif

#ifndef HOLD_CHUNK_US
#define HOLD_CHUNK_US 50000U
#endif

#ifndef DIR_SETUP_US
#define DIR_SETUP_US 20U
#endif

#ifndef SERVO_PWM_GPIO
#define SERVO_PWM_GPIO GPIO_NUM_3
#endif

#ifndef SERVO_PWM_HZ
#define SERVO_PWM_HZ 50U
#endif

#ifndef SERVO_MIN_ANGLE
#define SERVO_MIN_ANGLE 0U
#endif

#ifndef SERVO_MAX_ANGLE
#define SERVO_MAX_ANGLE 180U
#endif

#ifndef SERVO_DEFAULT_ANGLE
#define SERVO_DEFAULT_ANGLE 90U
#endif

#ifndef SERVO_MIN_PULSE_US
#define SERVO_MIN_PULSE_US 500U
#endif

#ifndef SERVO_MAX_PULSE_US
#define SERVO_MAX_PULSE_US 2500U
#endif

#ifndef CONFIG_STEP_MOTOR_DEVICE_NAME
#define CONFIG_STEP_MOTOR_DEVICE_NAME "esp32-step-motor"
#endif

#ifndef CONFIG_STEP_MOTOR_WIFI_SSID
#define CONFIG_STEP_MOTOR_WIFI_SSID ""
#endif

#ifndef CONFIG_STEP_MOTOR_WIFI_PASSWORD
#define CONFIG_STEP_MOTOR_WIFI_PASSWORD ""
#endif

#ifndef CONFIG_STEP_MOTOR_AP_SSID
#define CONFIG_STEP_MOTOR_AP_SSID "esp32-step-motor"
#endif

#ifndef CONFIG_STEP_MOTOR_AP_PASSWORD
#define CONFIG_STEP_MOTOR_AP_PASSWORD "stepmotor123"
#endif

static const char *TAG = "step_motor";
static const char *HOSTNAME = CONFIG_STEP_MOTOR_DEVICE_NAME;


typedef enum {
    MOTION_PROFILE_TIME = 0,
    MOTION_PROFILE_STEPS = 1,
} motion_profile_t;

typedef enum {
    MOTOR_ACTION_NONE = 0,
    MOTOR_ACTION_JOG_FORWARD,
    MOTOR_ACTION_JOG_REVERSE,
    MOTOR_ACTION_HOLD_FORWARD,
    MOTOR_ACTION_HOLD_REVERSE,
    MOTOR_ACTION_START_AUTO,
    MOTOR_ACTION_STOP,
} motor_action_t;

typedef enum {
    MOTOR_ACTIVITY_IDLE = 0,
    MOTOR_ACTIVITY_RUNNING,
    MOTOR_ACTIVITY_AUTO,
    MOTOR_ACTIVITY_HOLD,
} motor_activity_t;

typedef struct {
    motion_profile_t profile;
    uint32_t step_period_us;
    uint32_t rpm;
    uint32_t move_time_ms;
    uint32_t move_steps;
    uint32_t pause_ms;
    uint32_t dir_setup_us;
    bool auto_mode;
    bool hold_mode;
    bool running;
    motor_activity_t activity;
    motor_action_t pending_action;
    uint32_t current_step_period_us;
    uint32_t pwm_resolution_bits;
} motor_state_t;

typedef struct {
    gpio_num_t step_gpio;
    gpio_num_t dir_gpio;
    gpio_num_t en_gpio;
    ledc_channel_t channel;
    ledc_timer_t timer;
    TaskHandle_t task_handle;
} motor_hw_t;

typedef struct {
    motor_hw_t hw;
    motor_state_t state;
} motor_ctx_t;

typedef struct {
    bool wifi_sta_connected;
    bool wifi_ap_started;
    char wifi_sta_ip[16];
    char wifi_ap_ip[16];
    char wifi_mode[8];
    char wifi_ssid[33];
    bool wifi_emergency_ap;
    char hostname[32];
} app_state_t;

typedef struct {
    gpio_num_t pwm_gpio;
    ledc_channel_t channel;
    ledc_timer_t timer;
} servo_hw_t;

typedef struct {
    servo_hw_t hw;
    uint32_t angle;
    uint32_t target_angle;
    uint32_t current_pulse_us;
    uint32_t pwm_resolution_bits;
    bool enabled;
} servo_ctx_t;

static SemaphoreHandle_t g_state_mutex;
static httpd_handle_t g_http_server;
static esp_netif_t *g_sta_netif;
static esp_netif_t *g_ap_netif;
static esp_event_handler_instance_t g_wifi_handler_instance;
static esp_event_handler_instance_t g_ip_handler_instance;
static motor_ctx_t g_motors[2];
static servo_ctx_t g_servo;
static uint32_t step_frequency_hz_from_period(uint32_t step_period_us);
static uint32_t rpm_from_step_period_us(uint32_t step_period_us);
static uint32_t step_period_us_from_rpm(uint32_t rpm);
static uint32_t servo_angle_to_pulse_us(uint32_t angle);

typedef enum {
    WIFI_PREF_AP = 0,
    WIFI_PREF_STA = 1,
} wifi_pref_t;

typedef struct {
    wifi_pref_t preferred_mode;
    char sta_ssid[33];
    char sta_password[65];
} wifi_settings_t;

static wifi_settings_t g_wifi_settings;
static bool g_wifi_started;
static app_state_t g_state;

static void update_ap_ip_from_netif(void);

static const char *motion_profile_to_string(motion_profile_t profile)
{
    return (profile == MOTION_PROFILE_STEPS) ? "steps" : "time";
}

static const char *motor_action_to_string(motor_action_t action)
{
    switch (action) {
        case MOTOR_ACTION_JOG_FORWARD:
            return "forward";
        case MOTOR_ACTION_JOG_REVERSE:
            return "reverse";
        case MOTOR_ACTION_HOLD_FORWARD:
            return "hold_forward";
        case MOTOR_ACTION_HOLD_REVERSE:
            return "hold_reverse";
        case MOTOR_ACTION_START_AUTO:
            return "auto_start";
        case MOTOR_ACTION_STOP:
            return "stop";
        case MOTOR_ACTION_NONE:
        default:
            return "none";
    }
}

static const char *motor_activity_to_string(motor_activity_t activity)
{
    switch (activity) {
        case MOTOR_ACTIVITY_RUNNING:
            return "running";
        case MOTOR_ACTIVITY_AUTO:
            return "auto";
        case MOTOR_ACTIVITY_HOLD:
            return "hold";
        case MOTOR_ACTIVITY_IDLE:
        default:
            return "idle";
    }
}

static const char *bool_to_json(bool value)
{
    return value ? "true" : "false";
}

static const char *wifi_pref_to_string(wifi_pref_t pref)
{
    return (pref == WIFI_PREF_STA) ? "sta" : "ap";
}

static wifi_pref_t wifi_pref_from_string(const char *value)
{
    if (value == NULL) {
        return WIFI_PREF_AP;
    }

    if (strcmp(value, "sta") == 0 || strcmp(value, "client") == 0 || strcmp(value, "wifi") == 0) {
        return WIFI_PREF_STA;
    }

    return WIFI_PREF_AP;
}

static void wifi_settings_sync_state_locked(void)
{
    snprintf(g_state.wifi_mode, sizeof(g_state.wifi_mode), "%s", wifi_pref_to_string(g_wifi_settings.preferred_mode));
    snprintf(g_state.wifi_ssid, sizeof(g_state.wifi_ssid), "%s", g_wifi_settings.sta_ssid);
    g_state.wifi_emergency_ap = true;
}

static void wifi_settings_set_defaults(void)
{
    memset(&g_wifi_settings, 0, sizeof(g_wifi_settings));

    if (strlen(CONFIG_STEP_MOTOR_WIFI_SSID) > 0U) {
        g_wifi_settings.preferred_mode = WIFI_PREF_STA;
        snprintf(g_wifi_settings.sta_ssid, sizeof(g_wifi_settings.sta_ssid), "%s", CONFIG_STEP_MOTOR_WIFI_SSID);
        snprintf(g_wifi_settings.sta_password, sizeof(g_wifi_settings.sta_password), "%s", CONFIG_STEP_MOTOR_WIFI_PASSWORD);
    } else {
        g_wifi_settings.preferred_mode = WIFI_PREF_AP;
    }
}

static void copy_string_field(char *dst, size_t dst_len, const char *src)
{
    if (dst_len == 0) {
        return;
    }

    size_t n = strnlen(src, dst_len - 1);
    memcpy(dst, src, n);
    dst[n] = '\0';
}

static void wifi_settings_apply_to_state(void)
{
    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    wifi_settings_sync_state_locked();
    xSemaphoreGive(g_state_mutex);
}

static void wifi_settings_load_from_nvs(void)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open("wifi_cfg", NVS_READONLY, &handle);
    if (err != ESP_OK) {
        wifi_settings_apply_to_state();
        return;
    }

    uint8_t mode = (uint8_t)g_wifi_settings.preferred_mode;
    size_t len = sizeof(g_wifi_settings.sta_ssid);
    if (nvs_get_u8(handle, "mode", &mode) == ESP_OK) {
        g_wifi_settings.preferred_mode = (wifi_pref_t)mode;
    }

    len = sizeof(g_wifi_settings.sta_ssid);
    if (nvs_get_str(handle, "ssid", g_wifi_settings.sta_ssid, &len) != ESP_OK) {
        g_wifi_settings.sta_ssid[0] = '\0';
    }

    len = sizeof(g_wifi_settings.sta_password);
    if (nvs_get_str(handle, "password", g_wifi_settings.sta_password, &len) != ESP_OK) {
        g_wifi_settings.sta_password[0] = '\0';
    }

    nvs_close(handle);
    wifi_settings_apply_to_state();
}

static esp_err_t wifi_settings_save_to_nvs(void)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open("wifi_cfg", NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        return err;
    }

    err = nvs_set_u8(handle, "mode", (uint8_t)g_wifi_settings.preferred_mode);
    if (err == ESP_OK) {
        err = nvs_set_str(handle, "ssid", g_wifi_settings.sta_ssid);
    }
    if (err == ESP_OK) {
        err = nvs_set_str(handle, "password", g_wifi_settings.sta_password);
    }
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }

    nvs_close(handle);
    return err;
}

static bool wifi_should_use_sta(void)
{
    return g_wifi_settings.preferred_mode == WIFI_PREF_STA && g_wifi_settings.sta_ssid[0] != '\0';
}

static void wifi_build_ap_config(wifi_config_t *ap_config)
{
    memset(ap_config, 0, sizeof(*ap_config));
    copy_string_field((char *)ap_config->ap.ssid, sizeof(ap_config->ap.ssid), CONFIG_STEP_MOTOR_AP_SSID);
    copy_string_field((char *)ap_config->ap.password, sizeof(ap_config->ap.password), CONFIG_STEP_MOTOR_AP_PASSWORD);
    ap_config->ap.ssid_len = strlen(CONFIG_STEP_MOTOR_AP_SSID);
    ap_config->ap.channel = 1;
    ap_config->ap.authmode = (strlen(CONFIG_STEP_MOTOR_AP_PASSWORD) >= 8U) ? WIFI_AUTH_WPA2_PSK : WIFI_AUTH_OPEN;
    ap_config->ap.max_connection = 4;
    ap_config->ap.beacon_interval = 100;
}

static void wifi_build_sta_config(wifi_config_t *sta_config)
{
    memset(sta_config, 0, sizeof(*sta_config));
    copy_string_field((char *)sta_config->sta.ssid, sizeof(sta_config->sta.ssid), g_wifi_settings.sta_ssid);
    copy_string_field((char *)sta_config->sta.password, sizeof(sta_config->sta.password), g_wifi_settings.sta_password);
    sta_config->sta.threshold.authmode = (strlen(g_wifi_settings.sta_password) >= 8U) ? WIFI_AUTH_WPA2_PSK : WIFI_AUTH_OPEN;
    sta_config->sta.pmf_cfg.capable = true;
    sta_config->sta.pmf_cfg.required = false;
}

static void wifi_request_sta_connect(void)
{
    esp_err_t err = esp_wifi_connect();
    if (err == ESP_OK || err == ESP_ERR_WIFI_CONN) {
        return;
    }

    ESP_ERROR_CHECK(err);
}

static void wifi_apply_settings(bool stop_first)
{
    wifi_config_t ap_config;
    wifi_build_ap_config(&ap_config);

    if (stop_first && g_wifi_started) {
        ESP_ERROR_CHECK(esp_wifi_stop());
        g_wifi_started = false;
    }

    const bool use_sta = wifi_should_use_sta();
    ESP_ERROR_CHECK(esp_wifi_set_mode(use_sta ? WIFI_MODE_APSTA : WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));

    if (use_sta) {
        wifi_config_t sta_config;
        wifi_build_sta_config(&sta_config);
        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_config));
    }

    ESP_ERROR_CHECK(esp_wifi_start());
    g_wifi_started = true;
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

    if (use_sta) {
        wifi_request_sta_connect();
    }

    update_ap_ip_from_netif();
    wifi_settings_apply_to_state();
}

static esp_err_t send_json(httpd_req_t *req, const char *json);

static const char *partition_label_or_unknown(const esp_partition_t *partition)
{
    return (partition != NULL && partition->label[0] != '\0') ? partition->label : "unknown";
}

static void ota_reboot_task(void *arg)
{
    const uint32_t delay_ms = (uint32_t)(uintptr_t)arg;
    vTaskDelay(pdMS_TO_TICKS(delay_ms));
    esp_restart();
    vTaskDelete(NULL);
}

static void ota_schedule_reboot(uint32_t delay_ms)
{
    (void)xTaskCreate(ota_reboot_task, "ota_reboot", 2048, (void *)(uintptr_t)delay_ms, 5, NULL);
}

static void ota_mark_app_valid_if_pending(void)
{
    const esp_partition_t *running = esp_ota_get_running_partition();
    if (running == NULL) {
        return;
    }

    esp_ota_img_states_t ota_state;
    if (esp_ota_get_state_partition(running, &ota_state) == ESP_OK && ota_state == ESP_OTA_IMG_PENDING_VERIFY) {
        esp_err_t err = esp_ota_mark_app_valid_cancel_rollback();
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "Confirmed OTA image as valid");
        } else {
            ESP_LOGW(TAG, "Failed to confirm OTA image: %s", esp_err_to_name(err));
        }
    }
}

static void build_ota_json(char *buf, size_t len)
{
    const esp_app_desc_t *app_desc = esp_app_get_description();
    const esp_partition_t *running = esp_ota_get_running_partition();
    const esp_partition_t *next = esp_ota_get_next_update_partition(NULL);
    esp_ota_img_states_t ota_state;
    const bool pending_verify = (running != NULL && esp_ota_get_state_partition(running, &ota_state) == ESP_OK && ota_state == ESP_OTA_IMG_PENDING_VERIFY);
    const char *version = (app_desc != NULL) ? app_desc->version : "unknown";
    const char *running_label = partition_label_or_unknown(running);
    const char *next_label = partition_label_or_unknown(next);
    const uint32_t next_size = (next != NULL) ? (uint32_t)next->size : 0U;

    snprintf(buf, len,
             "{"
             "\"ota_supported\":%s,"
             "\"firmware_version\":\"%s\","
             "\"running_partition\":\"%s\","
             "\"next_partition\":\"%s\","
             "\"slot_size\":%" PRIu32 ","
             "\"pending_verify\":%s"
             "}",
             bool_to_json(next != NULL),
             version,
             running_label,
             next_label,
             next_size,
             bool_to_json(pending_verify));
}

static esp_err_t ota_get_handler(httpd_req_t *req)
{
    char json[512];
    build_ota_json(json, sizeof(json));
    return send_json(req, json);
}

static esp_err_t ota_post_handler(httpd_req_t *req)
{
    const esp_partition_t *update_partition = esp_ota_get_next_update_partition(NULL);
    if (update_partition == NULL) {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "no OTA partition available");
    }

    if (req->content_len == 0U) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "empty OTA payload");
    }

    if (req->content_len > update_partition->size) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "image larger than OTA partition");
    }

    esp_ota_handle_t ota_handle = 0;
    esp_err_t err = esp_ota_begin(update_partition, req->content_len, &ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_begin failed: %s", esp_err_to_name(err));
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "unable to start OTA update");
    }

    char buffer[1024];
    size_t remaining = req->content_len;
    while (remaining > 0U) {
        const size_t chunk = remaining < sizeof(buffer) ? remaining : sizeof(buffer);
        const int received = httpd_req_recv(req, buffer, chunk);
        if (received <= 0) {
            ESP_LOGE(TAG, "OTA receive failed: %d", received);
            esp_ota_abort(ota_handle);
            return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA upload interrupted");
        }

        err = esp_ota_write(ota_handle, buffer, (size_t)received);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "esp_ota_write failed: %s", esp_err_to_name(err));
            esp_ota_abort(ota_handle);
            return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA write failed");
        }

        remaining -= (size_t)received;
    }

    err = esp_ota_end(ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_end failed: %s", esp_err_to_name(err));
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA finalize failed");
    }

    err = esp_ota_set_boot_partition(update_partition);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_set_boot_partition failed: %s", esp_err_to_name(err));
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "unable to set boot partition");
    }

    char json[512];
    snprintf(json, sizeof(json), "{\"status\":\"ok\",\"message\":\"OTA update accepted\",\"rebooting_in_ms\":1500,\"target_partition\":\"%s\"}", partition_label_or_unknown(update_partition));
    esp_err_t send_err = send_json(req, json);
    if (send_err == ESP_OK) {
        ota_schedule_reboot(1500U);
    }
    return send_err;
}

static void state_init_defaults(void)
{
    memset(&g_state, 0, sizeof(g_state));
    g_state.wifi_sta_connected = false;
    g_state.wifi_ap_started = false;
    g_state.wifi_sta_ip[0] = '\0';
    g_state.wifi_ap_ip[0] = '\0';
    g_state.wifi_mode[0] = '\0';
    g_state.wifi_ssid[0] = '\0';
    g_state.wifi_emergency_ap = true;
    snprintf(g_state.hostname, sizeof(g_state.hostname), "%s", HOSTNAME);
    wifi_settings_apply_to_state();

    for (size_t i = 0; i < 2; ++i) {
        g_motors[i].state.profile = MOTION_PROFILE_TIME;
        g_motors[i].state.step_period_us = STEP_PERIOD_US;
        g_motors[i].state.rpm = rpm_from_step_period_us(STEP_PERIOD_US);
        g_motors[i].state.move_time_ms = MOVE_TIME_MS;
        g_motors[i].state.move_steps = STEP_COUNT;
        g_motors[i].state.pause_ms = PAUSE_MS;
        g_motors[i].state.dir_setup_us = DIR_SETUP_US;
        g_motors[i].state.auto_mode = false;
        g_motors[i].state.hold_mode = false;
        g_motors[i].state.running = false;
        g_motors[i].state.activity = MOTOR_ACTIVITY_IDLE;
        g_motors[i].state.pending_action = MOTOR_ACTION_NONE;
        g_motors[i].state.current_step_period_us = 0U;
        g_motors[i].state.pwm_resolution_bits = 0U;
    }

    g_servo.hw.pwm_gpio = SERVO_PWM_GPIO;
    g_servo.hw.channel = LEDC_CHANNEL_2;
    g_servo.hw.timer = LEDC_TIMER_2;
    g_servo.angle = SERVO_DEFAULT_ANGLE;
    g_servo.target_angle = SERVO_DEFAULT_ANGLE;
    g_servo.current_pulse_us = servo_angle_to_pulse_us(SERVO_DEFAULT_ANGLE);
    g_servo.pwm_resolution_bits = 0U;
    g_servo.enabled = false;
}

static app_state_t state_snapshot(void)
{
    app_state_t snapshot;
    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    snapshot = g_state;
    xSemaphoreGive(g_state_mutex);
    return snapshot;
}

static motor_state_t motor_snapshot(size_t motor_index)
{
    motor_state_t snapshot;
    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    snapshot = g_motors[motor_index].state;
    xSemaphoreGive(g_state_mutex);
    return snapshot;
}

static size_t motor_index_from_request(uint32_t motor)
{
    return (motor == 2U) ? 1U : 0U;
}

static void notify_motor_task(size_t motor_index)
{
    if (motor_index < 2U && g_motors[motor_index].hw.task_handle != NULL) {
        xTaskNotifyGive(g_motors[motor_index].hw.task_handle);
    }
}

static uint32_t clamp_u32(uint32_t value, uint32_t min_value, uint32_t max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static uint32_t step_frequency_hz_from_period(uint32_t step_period_us)
{
    step_period_us = clamp_u32(step_period_us, 50U, 1000000U);
    return 1000000U / step_period_us;
}

static uint32_t rpm_from_step_period_us(uint32_t step_period_us)
{
    step_period_us = clamp_u32(step_period_us, 50U, 1000000U);
    const uint64_t numerator = 60000000ULL;
    const uint64_t denominator = (uint64_t)step_period_us * (uint64_t)MOTOR_STEPS_PER_REV;
    if (denominator == 0ULL) {
        return 0U;
    }
    return (uint32_t)(numerator / denominator);
}

static uint32_t step_period_us_from_rpm(uint32_t rpm)
{
    rpm = clamp_u32(rpm, 1U, MOTOR_MAX_RPM);
    const uint64_t numerator = 60000000ULL;
    const uint64_t denominator = (uint64_t)rpm * (uint64_t)MOTOR_STEPS_PER_REV;
    if (denominator == 0ULL) {
        return STEP_PERIOD_US;
    }
    return (uint32_t)(numerator / denominator);
}

static uint32_t ledc_resolution_bits_for_frequency(uint32_t freq_hz)
{
    if (freq_hz <= 1000U) {
        return LEDC_TIMER_10_BIT;
    }
    if (freq_hz <= 5000U) {
        return LEDC_TIMER_9_BIT;
    }
    return LEDC_TIMER_8_BIT;
}

static void motor_enable(size_t motor_index, bool enable)
{
    const int level = EN_ACTIVE_LOW ? (enable ? 0 : 1) : (enable ? 1 : 0);
    gpio_set_level(g_motors[motor_index].hw.en_gpio, level);
}

static void motor_stop_output(size_t motor_index)
{
    motor_ctx_t *ctx = &g_motors[motor_index];
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, ctx->hw.channel, 0));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, ctx->hw.channel));
}

static void motor_apply_pwm_period(size_t motor_index, uint32_t step_period_us)
{
    motor_ctx_t *ctx = &g_motors[motor_index];
    step_period_us = clamp_u32(step_period_us, 50U, 1000000U);
    if (step_period_us == ctx->state.current_step_period_us && ctx->state.pwm_resolution_bits != 0U) {
        return;
    }

    const uint32_t freq_hz = step_frequency_hz_from_period(step_period_us);
    const uint32_t resolution_bits = ledc_resolution_bits_for_frequency(freq_hz);

    ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = (ledc_timer_bit_t)resolution_bits,
        .timer_num = ctx->hw.timer,
        .freq_hz = freq_hz,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));

    ctx->state.current_step_period_us = step_period_us;
    ctx->state.pwm_resolution_bits = resolution_bits;
}

static uint32_t servo_angle_to_pulse_us(uint32_t angle)
{
    angle = clamp_u32(angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
    const uint32_t angle_span = SERVO_MAX_ANGLE - SERVO_MIN_ANGLE;
    if (angle_span == 0U) {
        return SERVO_MIN_PULSE_US;
    }

    const uint32_t pulse_span = SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US;
    return SERVO_MIN_PULSE_US + (uint32_t)(((uint64_t)(angle - SERVO_MIN_ANGLE) * pulse_span) / angle_span);
}

static uint32_t servo_resolution_bits_for_frequency(uint32_t freq_hz)
{
    (void)freq_hz;
    return LEDC_TIMER_14_BIT;
}

static void servo_apply_pwm_frequency(void)
{
    const uint32_t freq_hz = SERVO_PWM_HZ;
    const uint32_t resolution_bits = servo_resolution_bits_for_frequency(freq_hz);

    ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = (ledc_timer_bit_t)resolution_bits,
        .timer_num = g_servo.hw.timer,
        .freq_hz = freq_hz,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));

    g_servo.pwm_resolution_bits = resolution_bits;
}

static void servo_apply_angle(uint32_t angle)
{
    g_servo.target_angle = clamp_u32(angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
    g_servo.angle = g_servo.target_angle;
    g_servo.current_pulse_us = servo_angle_to_pulse_us(g_servo.angle);

    const uint32_t max_duty = (1U << g_servo.pwm_resolution_bits) - 1U;
    const uint32_t duty = (uint32_t)(((uint64_t)g_servo.current_pulse_us * max_duty) / 20000ULL);
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, g_servo.hw.channel, duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, g_servo.hw.channel));
    g_servo.enabled = true;
}

static void motor_step_output_start(size_t motor_index)
{
    motor_ctx_t *ctx = &g_motors[motor_index];
    const uint32_t max_duty = (1U << ctx->state.pwm_resolution_bits) - 1U;
    const uint32_t duty = max_duty / 2U;
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, ctx->hw.channel, duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, ctx->hw.channel));
}

static motor_action_t take_pending_action(size_t motor_index)
{
    motor_action_t action;
    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    action = g_motors[motor_index].state.pending_action;
    g_motors[motor_index].state.pending_action = MOTOR_ACTION_NONE;
    xSemaphoreGive(g_state_mutex);
    return action;
}

static motor_action_t peek_pending_action(size_t motor_index)
{
    motor_action_t action;
    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    action = g_motors[motor_index].state.pending_action;
    xSemaphoreGive(g_state_mutex);
    return action;
}

static uint64_t motion_duration_us_from_state(const motor_state_t *state)
{
    if (state->profile == MOTION_PROFILE_STEPS) {
        return (uint64_t)state->move_steps * (uint64_t)state->step_period_us;
    }
    return (uint64_t)state->move_time_ms * 1000ULL;
}

static bool wait_abortable_us(size_t motor_index, uint64_t duration_us)
{
    const uint64_t slice_us = 20000ULL;
    const int64_t end_time = esp_timer_get_time() + (int64_t)duration_us;

    while (true) {
        if (peek_pending_action(motor_index) != MOTOR_ACTION_NONE) {
            return false;
        }

        const int64_t now = esp_timer_get_time();
        if (now >= end_time) {
            return true;
        }

        const uint64_t remaining = (uint64_t)(end_time - now);
        if (remaining > slice_us) {
            vTaskDelay(pdMS_TO_TICKS(20));
        } else {
            esp_rom_delay_us((uint32_t)remaining);
        }
    }
}

static bool motor_run_segment(size_t motor_index, bool forward, uint64_t duration_us, uint32_t dir_setup_us)
{
    if (duration_us == 0U) {
        return true;
    }

    gpio_set_level(g_motors[motor_index].hw.dir_gpio, forward ? 1 : 0);
    esp_rom_delay_us(dir_setup_us);

    motor_step_output_start(motor_index);
    const bool completed = wait_abortable_us(motor_index, duration_us);
    motor_stop_output(motor_index);
    return completed;
}

static bool motion_wait_pause(size_t motor_index, uint32_t pause_ms)
{
    return wait_abortable_us(motor_index, (uint64_t)pause_ms * 1000ULL);
}

static void set_activity_idle(motor_state_t *state)
{
    state->activity = MOTOR_ACTIVITY_IDLE;
    state->running = false;
    state->auto_mode = false;
    state->hold_mode = false;
}

static void set_activity_running(motor_state_t *state)
{
    state->activity = MOTOR_ACTIVITY_RUNNING;
    state->running = true;
    state->auto_mode = false;
    state->hold_mode = false;
}

static void set_activity_auto(motor_state_t *state)
{
    state->activity = MOTOR_ACTIVITY_AUTO;
    state->running = true;
    state->auto_mode = true;
    state->hold_mode = false;
}

static void set_activity_hold(motor_state_t *state)
{
    state->activity = MOTOR_ACTIVITY_HOLD;
    state->running = true;
    state->auto_mode = false;
    state->hold_mode = true;
}


static void motor_task(void *arg)
{
    const size_t motor_index = (size_t)(uintptr_t)arg;

    while (true) {
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100));

        motor_action_t action = take_pending_action(motor_index);
        if (action == MOTOR_ACTION_NONE) {
            const motor_state_t snapshot = motor_snapshot(motor_index);
            if (snapshot.auto_mode) {
                action = MOTOR_ACTION_START_AUTO;
            } else {
                continue;
            }
        }

        if (action == MOTOR_ACTION_STOP) {
            motor_stop_output(motor_index);
            xSemaphoreTake(g_state_mutex, portMAX_DELAY);
            set_activity_idle(&g_motors[motor_index].state);
            xSemaphoreGive(g_state_mutex);
            continue;
        }

        if (action == MOTOR_ACTION_JOG_FORWARD || action == MOTOR_ACTION_JOG_REVERSE) {
            const motor_state_t snapshot = motor_snapshot(motor_index);
            motor_apply_pwm_period(motor_index, snapshot.step_period_us);

            xSemaphoreTake(g_state_mutex, portMAX_DELAY);
            set_activity_running(&g_motors[motor_index].state);
            xSemaphoreGive(g_state_mutex);

            const uint64_t duration_us = motion_duration_us_from_state(&snapshot);
            (void)motor_run_segment(motor_index, action == MOTOR_ACTION_JOG_FORWARD, duration_us, snapshot.dir_setup_us);

            motor_stop_output(motor_index);
            xSemaphoreTake(g_state_mutex, portMAX_DELAY);
            set_activity_idle(&g_motors[motor_index].state);
            xSemaphoreGive(g_state_mutex);
            continue;
        }

        if (action == MOTOR_ACTION_HOLD_FORWARD || action == MOTOR_ACTION_HOLD_REVERSE) {
            const bool forward = (action == MOTOR_ACTION_HOLD_FORWARD);

            while (true) {
                motor_state_t snapshot = motor_snapshot(motor_index);
                if (!snapshot.hold_mode) {
                    break;
                }

                motor_apply_pwm_period(motor_index, snapshot.step_period_us);

                xSemaphoreTake(g_state_mutex, portMAX_DELAY);
                set_activity_hold(&g_motors[motor_index].state);
                xSemaphoreGive(g_state_mutex);

                if (!motor_run_segment(motor_index, forward, HOLD_CHUNK_US, snapshot.dir_setup_us)) {
                    break;
                }
            }

            motor_stop_output(motor_index);
            xSemaphoreTake(g_state_mutex, portMAX_DELAY);
            set_activity_idle(&g_motors[motor_index].state);
            xSemaphoreGive(g_state_mutex);
            continue;
        }

        if (action == MOTOR_ACTION_START_AUTO) {
            while (true) {
                motor_state_t snapshot = motor_snapshot(motor_index);
                if (!snapshot.auto_mode) {
                    break;
                }

                motor_apply_pwm_period(motor_index, snapshot.step_period_us);

                xSemaphoreTake(g_state_mutex, portMAX_DELAY);
                set_activity_auto(&g_motors[motor_index].state);
                xSemaphoreGive(g_state_mutex);

                const uint64_t duration_us = motion_duration_us_from_state(&snapshot);
                if (!motor_run_segment(motor_index, true, duration_us, snapshot.dir_setup_us)) {
                    break;
                }

                if (!motion_wait_pause(motor_index, snapshot.pause_ms)) {
                    break;
                }

                snapshot = motor_snapshot(motor_index);
                if (!snapshot.auto_mode) {
                    break;
                }

                motor_apply_pwm_period(motor_index, snapshot.step_period_us);
                if (!motor_run_segment(motor_index, false, duration_us, snapshot.dir_setup_us)) {
                    break;
                }

                if (!motion_wait_pause(motor_index, snapshot.pause_ms)) {
                    break;
                }
            }

            motor_stop_output(motor_index);
            xSemaphoreTake(g_state_mutex, portMAX_DELAY);
            set_activity_idle(&g_motors[motor_index].state);
            xSemaphoreGive(g_state_mutex);
            continue;
        }
    }
}

static bool json_get_string_field(const char *body, const char *key, char *out, size_t out_len)
{
    char pattern[64];
    snprintf(pattern, sizeof(pattern), "\"%s\"", key);
    const char *p = strstr(body, pattern);
    if (p == NULL) {
        return false;
    }

    p = strchr(p + strlen(pattern), ':');
    if (p == NULL) {
        return false;
    }

    p++;
    while (*p != '\0' && isspace((unsigned char)*p)) {
        p++;
    }

    if (*p != '"') {
        return false;
    }

    p++;
    const char *end = strchr(p, '"');
    if (end == NULL) {
        return false;
    }

    size_t len = (size_t)(end - p);
    if (len >= out_len) {
        len = out_len - 1U;
    }

    memcpy(out, p, len);
    out[len] = '\0';
    return true;
}

static bool json_get_uint32_field(const char *body, const char *key, uint32_t *out)
{
    char pattern[64];
    snprintf(pattern, sizeof(pattern), "\"%s\"", key);
    const char *p = strstr(body, pattern);
    if (p == NULL) {
        return false;
    }

    p = strchr(p + strlen(pattern), ':');
    if (p == NULL) {
        return false;
    }

    p++;
    while (*p != '\0' && isspace((unsigned char)*p)) {
        p++;
    }

    char *end = NULL;
    unsigned long value = strtoul(p, &end, 10);
    if (end == p) {
        return false;
    }

    *out = (uint32_t)value;
    return true;
}

static char *read_request_body(httpd_req_t *req)
{
    const size_t len = req->content_len;
    if (len == 0U || len > 1024U) {
        return NULL;
    }

    char *body = calloc(1U, len + 1U);
    if (body == NULL) {
        return NULL;
    }

    size_t received = 0U;
    while (received < len) {
        int ret = httpd_req_recv(req, body + received, len - received);
        if (ret <= 0) {
            free(body);
            return NULL;
        }
        received += (size_t)ret;
    }

    body[len] = '\0';
    return body;
}

static esp_err_t send_json(httpd_req_t *req, const char *json)
{
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, json, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t send_text(httpd_req_t *req, const char *text)
{
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, text, HTTPD_RESP_USE_STRLEN);
}

static void build_motor_json(char *buf, size_t len, size_t motor_index)
{
    const motor_state_t s = motor_snapshot(motor_index);
    const uint64_t duration_us = motion_duration_us_from_state(&s);
    const uint32_t move_duration_ms = (uint32_t)((duration_us + 999ULL) / 1000ULL);
    const uint32_t freq_hz = step_frequency_hz_from_period(s.step_period_us);

    snprintf(buf, len,
             "{"
             "\"id\":%u,"
             "\"profile\":\"%s\","
             "\"auto_mode\":%s,"
             "\"hold_mode\":%s,"
             "\"running\":%s,"
             "\"activity\":\"%s\","
             "\"pending_action\":\"%s\","
             "\"step_gpio\":%d,"
             "\"dir_gpio\":%d,"
             "\"en_gpio\":%d,"
             "\"step_period_us\":%" PRIu32 ","
             "\"step_frequency_hz\":%" PRIu32 ","
             "\"rpm\":%" PRIu32 ","
             "\"move_time_ms\":%" PRIu32 ","
             "\"move_steps\":%" PRIu32 ","
             "\"move_duration_ms\":%" PRIu32 ","
             "\"pause_ms\":%" PRIu32 ","
             "\"dir_setup_us\":%" PRIu32 "}",
             (unsigned)(motor_index + 1U),
             motion_profile_to_string(s.profile),
             bool_to_json(s.auto_mode),
             bool_to_json(s.hold_mode),
             bool_to_json(s.running),
             motor_activity_to_string(s.activity),
             motor_action_to_string(s.pending_action),
             g_motors[motor_index].hw.step_gpio,
             g_motors[motor_index].hw.dir_gpio,
             g_motors[motor_index].hw.en_gpio,
             s.step_period_us,
             freq_hz,
             s.rpm,
             s.move_time_ms,
             s.move_steps,
             move_duration_ms,
             s.pause_ms,
             s.dir_setup_us);
}

static void build_servo_json(char *buf, size_t len)
{
    snprintf(buf, len,
             "{"
             "\"gpio\":%d,"
             "\"enabled\":%s,"
             "\"angle\":%" PRIu32 ","
             "\"target_angle\":%" PRIu32 ","
             "\"min_angle\":%u,"
             "\"max_angle\":%u,"
             "\"pulse_min_us\":%u,"
             "\"pulse_max_us\":%u"
             "}",
             g_servo.hw.pwm_gpio,
             bool_to_json(g_servo.enabled),
             g_servo.angle,
             g_servo.target_angle,
             SERVO_MIN_ANGLE,
             SERVO_MAX_ANGLE,
             SERVO_MIN_PULSE_US,
             SERVO_MAX_PULSE_US);
}

static void build_state_json(char *buf, size_t len)
{
    const app_state_t s = state_snapshot();
    const esp_app_desc_t *app_desc = esp_app_get_description();
    const esp_partition_t *running_partition = esp_ota_get_running_partition();
    esp_ota_img_states_t ota_state;
    const bool ota_pending_verify = (running_partition != NULL && esp_ota_get_state_partition(running_partition, &ota_state) == ESP_OK && ota_state == ESP_OTA_IMG_PENDING_VERIFY);
    const char *firmware_version = (app_desc != NULL) ? app_desc->version : "unknown";
    const char *running_partition_label = partition_label_or_unknown(running_partition);
    char motor1[512];
    char motor2[512];
    char servo[256];
    build_motor_json(motor1, sizeof(motor1), 0U);
    build_motor_json(motor2, sizeof(motor2), 1U);
    build_servo_json(servo, sizeof(servo));

    snprintf(buf, len,
             "{"
             "\"motor1\":%s,"
             "\"motor2\":%s,"
             "\"servo\":%s,"
             "\"wifi_sta_connected\":%s,"
             "\"wifi_sta_ip\":\"%s\","
             "\"wifi_ap_started\":%s,"
             "\"wifi_ap_ip\":\"%s\","
             "\"wifi_mode\":\"%s\","
             "\"wifi_ssid\":\"%s\","
             "\"wifi_emergency_ap\":%s,"
             "\"hostname\":\"%s\","
             "\"firmware_version\":\"%s\","
             "\"running_partition\":\"%s\","
             "\"ota_pending_verify\":%s"
             "}",
             motor1,
             motor2,
             servo,
             bool_to_json(s.wifi_sta_connected),
             s.wifi_sta_ip,
             bool_to_json(s.wifi_ap_started),
             s.wifi_ap_ip,
             s.wifi_mode,
             s.wifi_ssid,
             bool_to_json(s.wifi_emergency_ap),
             s.hostname,
             firmware_version,
             running_partition_label,
             bool_to_json(ota_pending_verify));
}

static void build_wifi_json(char *buf, size_t len)
{
    const app_state_t s = state_snapshot();
    snprintf(buf, len,
             "{"
             "\"mode\":\"%s\","
             "\"ssid\":\"%s\","
             "\"sta_connected\":%s,"
             "\"sta_ip\":\"%s\","
             "\"ap_started\":%s,"
             "\"ap_ip\":\"%s\","
             "\"emergency_ap\":%s,"
             "\"hostname\":\"%s\""
             "}",
             s.wifi_mode,
             s.wifi_ssid,
             bool_to_json(s.wifi_sta_connected),
             s.wifi_sta_ip,
             bool_to_json(s.wifi_ap_started),
             s.wifi_ap_ip,
             bool_to_json(s.wifi_emergency_ap),
             s.hostname);
}

static esp_err_t root_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, INDEX_HTML, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t health_get_handler(httpd_req_t *req)
{
    return send_text(req, "ok\n");
}

static esp_err_t state_get_handler(httpd_req_t *req)
{
    char json[2048];
    build_state_json(json, sizeof(json));
    return send_json(req, json);
}

static esp_err_t config_post_handler(httpd_req_t *req)
{
    char *body = read_request_body(req);
    if (body == NULL) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid request body");
    }

    uint32_t motor_value = 1U;
    (void)json_get_uint32_field(body, "motor", &motor_value);
    const size_t motor_index = motor_index_from_request(motor_value);

    motor_state_t snapshot = motor_snapshot(motor_index);
    uint32_t value;
    char str[64];

    if (json_get_string_field(body, "profile", str, sizeof(str))) {
        snapshot.profile = (strcmp(str, "steps") == 0) ? MOTION_PROFILE_STEPS : MOTION_PROFILE_TIME;
    }

    if (json_get_uint32_field(body, "step_period_us", &value)) {
        snapshot.step_period_us = clamp_u32(value, 50U, 1000000U);
        snapshot.rpm = rpm_from_step_period_us(snapshot.step_period_us);
    }

    if (json_get_uint32_field(body, "rpm", &value)) {
        snapshot.rpm = clamp_u32(value, 1U, MOTOR_MAX_RPM);
        snapshot.step_period_us = step_period_us_from_rpm(snapshot.rpm);
    }

    if (json_get_uint32_field(body, "move_time_ms", &value)) {
        snapshot.move_time_ms = value;
    }

    if (json_get_uint32_field(body, "move_steps", &value)) {
        snapshot.move_steps = value;
    }

    if (json_get_uint32_field(body, "pause_ms", &value)) {
        snapshot.pause_ms = value;
    }

    if (json_get_uint32_field(body, "dir_setup_us", &value)) {
        snapshot.dir_setup_us = value;
    }

    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    g_motors[motor_index].state.profile = snapshot.profile;
    g_motors[motor_index].state.step_period_us = snapshot.step_period_us;
    g_motors[motor_index].state.rpm = snapshot.rpm;
    g_motors[motor_index].state.move_time_ms = snapshot.move_time_ms;
    g_motors[motor_index].state.move_steps = snapshot.move_steps;
    g_motors[motor_index].state.pause_ms = snapshot.pause_ms;
    g_motors[motor_index].state.dir_setup_us = snapshot.dir_setup_us;
    xSemaphoreGive(g_state_mutex);

    free(body);
    notify_motor_task(motor_index);

    char json[2048];
    build_state_json(json, sizeof(json));
    return send_json(req, json);
}

static esp_err_t control_post_handler(httpd_req_t *req)
{
    char *body = read_request_body(req);
    if (body == NULL) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid request body");
    }

    uint32_t motor_value = 1U;
    (void)json_get_uint32_field(body, "motor", &motor_value);
    const size_t motor_index = motor_index_from_request(motor_value);

    char action[32];
    if (!json_get_string_field(body, "action", action, sizeof(action))) {
        free(body);
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "missing action");
    }

    motor_action_t pending = MOTOR_ACTION_NONE;
    bool auto_mode = false;
    bool hold_mode = false;

    if (strcmp(action, "forward") == 0) {
        pending = MOTOR_ACTION_JOG_FORWARD;
    } else if (strcmp(action, "reverse") == 0) {
        pending = MOTOR_ACTION_JOG_REVERSE;
    } else if (strcmp(action, "hold_forward") == 0) {
        pending = MOTOR_ACTION_HOLD_FORWARD;
        hold_mode = true;
    } else if (strcmp(action, "hold_reverse") == 0) {
        pending = MOTOR_ACTION_HOLD_REVERSE;
        hold_mode = true;
    } else if (strcmp(action, "stop") == 0) {
        pending = MOTOR_ACTION_STOP;
    } else if (strcmp(action, "auto_start") == 0) {
        pending = MOTOR_ACTION_START_AUTO;
        auto_mode = true;
    } else if (strcmp(action, "auto_stop") == 0) {
        pending = MOTOR_ACTION_STOP;
    } else {
        free(body);
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "unknown action");
    }

    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    g_motors[motor_index].state.pending_action = pending;
    g_motors[motor_index].state.auto_mode = auto_mode;
    if (hold_mode) {
        g_motors[motor_index].state.hold_mode = true;
    }
    if (pending == MOTOR_ACTION_STOP) {
        g_motors[motor_index].state.auto_mode = false;
        g_motors[motor_index].state.hold_mode = false;
    }
    xSemaphoreGive(g_state_mutex);

    free(body);
    notify_motor_task(motor_index);

    char json[2048];
    build_state_json(json, sizeof(json));
    return send_json(req, json);
}

static esp_err_t servo_get_handler(httpd_req_t *req)
{
    char json[256];
    build_servo_json(json, sizeof(json));
    return send_json(req, json);
}

static esp_err_t servo_post_handler(httpd_req_t *req)
{
    char *body = read_request_body(req);
    if (body == NULL) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid request body");
    }

    uint32_t angle = SERVO_DEFAULT_ANGLE;
    char action[32];
    if (json_get_string_field(body, "action", action, sizeof(action))) {
        if (strcmp(action, "center") == 0) {
            angle = 90U;
        } else if (strcmp(action, "min") == 0) {
            angle = SERVO_MIN_ANGLE;
        } else if (strcmp(action, "max") == 0) {
            angle = SERVO_MAX_ANGLE;
        }
    }
    (void)json_get_uint32_field(body, "angle", &angle);

    angle = clamp_u32(angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);

    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    servo_apply_angle(angle);
    xSemaphoreGive(g_state_mutex);

    free(body);

    char json[2048];
    build_state_json(json, sizeof(json));
    return send_json(req, json);
}

static esp_err_t wifi_get_handler(httpd_req_t *req)
{

    char json[1024];
    build_wifi_json(json, sizeof(json));
    return send_json(req, json);
}

static esp_err_t wifi_post_handler(httpd_req_t *req)
{
    char *body = read_request_body(req);
    if (body == NULL) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid request body");
    }

    wifi_settings_t settings;
    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    settings = g_wifi_settings;
    xSemaphoreGive(g_state_mutex);

    char str[64];
    if (json_get_string_field(body, "mode", str, sizeof(str))) {
        settings.preferred_mode = wifi_pref_from_string(str);
    }

    if (json_get_string_field(body, "ssid", str, sizeof(str)) && str[0] != '\0') {
        copy_string_field(settings.sta_ssid, sizeof(settings.sta_ssid), str);
    }

    if (json_get_string_field(body, "password", str, sizeof(str)) && str[0] != '\0') {
        copy_string_field(settings.sta_password, sizeof(settings.sta_password), str);
    }

    if (settings.preferred_mode == WIFI_PREF_STA && settings.sta_ssid[0] == '\0') {
        free(body);
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "ssid required for station mode");
    }

    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    g_wifi_settings = settings;
    wifi_settings_sync_state_locked();
    xSemaphoreGive(g_state_mutex);

    esp_err_t save_err = wifi_settings_save_to_nvs();
    if (save_err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to save Wi-Fi settings to NVS: %s", esp_err_to_name(save_err));
    }

    wifi_apply_settings(true);
    free(body);

    char json[1024];
    build_wifi_json(json, sizeof(json));
    return send_json(req, json);
}

static esp_err_t options_handler(httpd_req_t *req)
{
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET, POST, OPTIONS");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type");
    return httpd_resp_send(req, "", 0);
}

static void register_uri(httpd_handle_t server, const char *uri, httpd_method_t method, esp_err_t (*handler)(httpd_req_t *))
{
    httpd_uri_t config = {
        .uri = uri,
        .method = method,
        .handler = handler,
        .user_ctx = NULL,
    };
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &config));
}

static void start_http_server(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = 8192;
    config.max_uri_handlers = 20;
    config.lru_purge_enable = true;

    ESP_ERROR_CHECK(httpd_start(&g_http_server, &config));
    register_uri(g_http_server, "/", HTTP_GET, root_get_handler);
    register_uri(g_http_server, "/", HTTP_OPTIONS, options_handler);
    register_uri(g_http_server, "/api/health", HTTP_GET, health_get_handler);
    register_uri(g_http_server, "/api/health", HTTP_OPTIONS, options_handler);
    register_uri(g_http_server, "/api/state", HTTP_GET, state_get_handler);
    register_uri(g_http_server, "/api/state", HTTP_OPTIONS, options_handler);
    register_uri(g_http_server, "/api/config", HTTP_POST, config_post_handler);
    register_uri(g_http_server, "/api/config", HTTP_OPTIONS, options_handler);
    register_uri(g_http_server, "/api/control", HTTP_POST, control_post_handler);
    register_uri(g_http_server, "/api/control", HTTP_OPTIONS, options_handler);
    register_uri(g_http_server, "/api/servo", HTTP_GET, servo_get_handler);
    register_uri(g_http_server, "/api/servo", HTTP_OPTIONS, options_handler);
    register_uri(g_http_server, "/api/servo", HTTP_POST, servo_post_handler);
    register_uri(g_http_server, "/api/wifi", HTTP_GET, wifi_get_handler);
    register_uri(g_http_server, "/api/wifi", HTTP_OPTIONS, options_handler);
    register_uri(g_http_server, "/api/wifi", HTTP_POST, wifi_post_handler);
    register_uri(g_http_server, "/api/ota", HTTP_GET, ota_get_handler);
    register_uri(g_http_server, "/api/ota", HTTP_OPTIONS, options_handler);
    register_uri(g_http_server, "/api/ota", HTTP_POST, ota_post_handler);
}


static void update_sta_ip(const char *ip)
{
    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    g_state.wifi_sta_connected = true;
    snprintf(g_state.wifi_sta_ip, sizeof(g_state.wifi_sta_ip), "%s", ip);
    xSemaphoreGive(g_state_mutex);
}

static void update_sta_disconnected(void)
{
    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    g_state.wifi_sta_connected = false;
    g_state.wifi_sta_ip[0] = '\0';
    xSemaphoreGive(g_state_mutex);
}

static void update_ap_ip_from_netif(void)
{
    if (g_ap_netif == NULL) {
        return;
    }

    esp_netif_ip_info_t ip_info;
    if (esp_netif_get_ip_info(g_ap_netif, &ip_info) == ESP_OK) {
        char ip[16];
        ip4addr_ntoa_r((const ip4_addr_t *)&ip_info.ip, ip, sizeof(ip));
        xSemaphoreTake(g_state_mutex, portMAX_DELAY);
        g_state.wifi_ap_started = true;
        snprintf(g_state.wifi_ap_ip, sizeof(g_state.wifi_ap_ip), "%s", ip);
        xSemaphoreGive(g_state_mutex);
    }
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    (void)arg;

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        if (wifi_should_use_sta()) {
            wifi_request_sta_connect();
        }
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        update_sta_disconnected();
        if (wifi_should_use_sta()) {
            ESP_LOGW(TAG, "Wi-Fi disconnected, reconnecting...");
            wifi_request_sta_connect();
        }
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_START) {
        update_ap_ip_from_netif();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        const ip_event_got_ip_t *event = (const ip_event_got_ip_t *)event_data;
        char ip[16];
        ip4addr_ntoa_r((const ip4_addr_t *)&event->ip_info.ip, ip, sizeof(ip));
        update_sta_ip(ip);
        ESP_LOGI(TAG, "Wi-Fi connected: %s", ip);
    }
}

static void wifi_start(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    g_sta_netif = esp_netif_create_default_wifi_sta();
    g_ap_netif = esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, &g_wifi_handler_instance));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, &g_ip_handler_instance));

    wifi_settings_set_defaults();
    wifi_settings_load_from_nvs();
    wifi_apply_settings(false);

    if (!wifi_should_use_sta()) {
        ESP_LOGW(TAG, "No Wi-Fi station SSID configured. Running AP emergency mode only.");
    }
}

static void motor_setup(void)
{
    g_motors[0].hw.step_gpio = MOTOR1_STEP_GPIO;
    g_motors[0].hw.dir_gpio = MOTOR1_DIR_GPIO;
    g_motors[0].hw.en_gpio = MOTOR1_EN_GPIO;
    g_motors[0].hw.channel = LEDC_CHANNEL_0;
    g_motors[0].hw.timer = LEDC_TIMER_0;
    g_motors[0].hw.task_handle = NULL;

    g_motors[1].hw.step_gpio = MOTOR2_STEP_GPIO;
    g_motors[1].hw.dir_gpio = MOTOR2_DIR_GPIO;
    g_motors[1].hw.en_gpio = MOTOR2_EN_GPIO;
    g_motors[1].hw.channel = LEDC_CHANNEL_1;
    g_motors[1].hw.timer = LEDC_TIMER_1;
    g_motors[1].hw.task_handle = NULL;

    gpio_config_t io = {
        .pin_bit_mask = (1ULL << g_motors[0].hw.step_gpio) | (1ULL << g_motors[0].hw.dir_gpio) | (1ULL << g_motors[0].hw.en_gpio) |
                        (1ULL << g_motors[1].hw.step_gpio) | (1ULL << g_motors[1].hw.dir_gpio) | (1ULL << g_motors[1].hw.en_gpio) |
                        (1ULL << g_servo.hw.pwm_gpio),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io));

    for (size_t i = 0; i < 2; ++i) {
        gpio_set_level(g_motors[i].hw.step_gpio, 0);
        gpio_set_level(g_motors[i].hw.dir_gpio, 0);
        motor_enable(i, false);
    }

    ledc_channel_config_t channel_1 = {
        .gpio_num = g_motors[0].hw.step_gpio,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = g_motors[0].hw.channel,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = g_motors[0].hw.timer,
        .duty = 0,
        .hpoint = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&channel_1));

    ledc_channel_config_t channel_2 = {
        .gpio_num = g_motors[1].hw.step_gpio,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = g_motors[1].hw.channel,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = g_motors[1].hw.timer,
        .duty = 0,
        .hpoint = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&channel_2));

    servo_apply_pwm_frequency();
    ledc_channel_config_t servo_channel = {
        .gpio_num = g_servo.hw.pwm_gpio,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = g_servo.hw.channel,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = g_servo.hw.timer,
        .duty = 0,
        .hpoint = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&servo_channel));
    servo_apply_angle(SERVO_DEFAULT_ANGLE);

    for (size_t i = 0; i < 2; ++i) {
        motor_apply_pwm_period(i, g_motors[i].state.step_period_us);
        motor_stop_output(i);
    }

    ESP_LOGI(TAG, "Motor 1 pins STEP=%d DIR=%d EN=%d", MOTOR1_STEP_GPIO, MOTOR1_DIR_GPIO, MOTOR1_EN_GPIO);
    ESP_LOGI(TAG, "Motor 2 pins STEP=%d DIR=%d EN=%d", MOTOR2_STEP_GPIO, MOTOR2_DIR_GPIO, MOTOR2_EN_GPIO);
    ESP_LOGI(TAG, "Servo pin PWM=%d angle=%" PRIu32 "..%" PRIu32 " default=%" PRIu32, SERVO_PWM_GPIO, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE, SERVO_DEFAULT_ANGLE);
    ESP_LOGI(TAG, "Enable pin is %s", EN_ACTIVE_LOW ? "active-low" : "active-high");
    ESP_LOGI(TAG, "Defaults: motor1 profile=%s rpm=%" PRIu32 " step_period=%" PRIu32 " us move_time=%" PRIu32 " ms move_steps=%" PRIu32 " pause=%" PRIu32 " ms",
             motion_profile_to_string(g_motors[0].state.profile), g_motors[0].state.rpm, g_motors[0].state.step_period_us, g_motors[0].state.move_time_ms,
             g_motors[0].state.move_steps, g_motors[0].state.pause_ms);
    ESP_LOGI(TAG, "Defaults: motor2 profile=%s rpm=%" PRIu32 " step_period=%" PRIu32 " us move_time=%" PRIu32 " ms move_steps=%" PRIu32 " pause=%" PRIu32 " ms",
             motion_profile_to_string(g_motors[1].state.profile), g_motors[1].state.rpm, g_motors[1].state.step_period_us, g_motors[1].state.move_time_ms,
             g_motors[1].state.move_steps, g_motors[1].state.pause_ms);
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    } else {
        ESP_ERROR_CHECK(ret);
    }

    g_state_mutex = xSemaphoreCreateMutex();
    configASSERT(g_state_mutex != NULL);

    state_init_defaults();
    motor_setup();
    wifi_start();
    start_http_server();
    ota_mark_app_valid_if_pending();

    xTaskCreate(motor_task, "motor1_task", 4096, (void *)0, 5, &g_motors[0].hw.task_handle);
    xTaskCreate(motor_task, "motor2_task", 4096, (void *)1, 5, &g_motors[1].hw.task_handle);
    motor_enable(0, true);
    motor_enable(1, true);

    ESP_LOGI(TAG, "HTTP server ready: http://%s/", HOSTNAME);
    ESP_LOGI(TAG, "API endpoints: GET /api/health, GET /api/state, POST /api/config, POST /api/control, GET /api/servo, POST /api/servo, GET /api/wifi, POST /api/wifi, GET /api/ota, POST /api/ota");
}
