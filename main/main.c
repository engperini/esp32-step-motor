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

#ifndef STEP_GPIO
#define STEP_GPIO GPIO_NUM_4
#endif

#ifndef DIR_GPIO
#define DIR_GPIO GPIO_NUM_5
#endif

#ifndef EN_GPIO
#define EN_GPIO GPIO_NUM_6
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

#ifndef DIR_SETUP_US
#define DIR_SETUP_US 20U
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

static const ledc_mode_t LEDC_SPEED_MODE = LEDC_LOW_SPEED_MODE;
static const ledc_timer_t LEDC_TIMER_NUM = LEDC_TIMER_0;
static const ledc_channel_t LEDC_CHANNEL_NUM = LEDC_CHANNEL_0;

typedef enum {
    MOTION_PROFILE_TIME = 0,
    MOTION_PROFILE_STEPS = 1,
} motion_profile_t;

typedef enum {
    MOTOR_ACTION_NONE = 0,
    MOTOR_ACTION_JOG_FORWARD,
    MOTOR_ACTION_JOG_REVERSE,
    MOTOR_ACTION_START_AUTO,
    MOTOR_ACTION_STOP,
} motor_action_t;

typedef enum {
    MOTOR_ACTIVITY_IDLE = 0,
    MOTOR_ACTIVITY_RUNNING,
    MOTOR_ACTIVITY_AUTO,
} motor_activity_t;

typedef struct {
    motion_profile_t profile;
    uint32_t step_period_us;
    uint32_t move_time_ms;
    uint32_t move_steps;
    uint32_t pause_ms;
    uint32_t dir_setup_us;
    bool auto_mode;
    bool running;
    motor_activity_t activity;
    motor_action_t pending_action;
    bool wifi_sta_connected;
    bool wifi_ap_started;
    char wifi_sta_ip[16];
    char wifi_ap_ip[16];
    char wifi_mode[8];
    char wifi_ssid[33];
    bool wifi_emergency_ap;
    char hostname[32];
} app_state_t;

static SemaphoreHandle_t g_state_mutex;
static TaskHandle_t g_motor_task_handle;
static httpd_handle_t g_http_server;
static esp_netif_t *g_sta_netif;
static esp_netif_t *g_ap_netif;
static esp_event_handler_instance_t g_wifi_handler_instance;
static esp_event_handler_instance_t g_ip_handler_instance;
static uint32_t g_current_step_period_us;
static uint32_t g_pwm_resolution_bits;

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

static const char INDEX_HTML[] =
"<!doctype html>\n"
"<html>\n"
"<head>\n"
"  <meta charset='utf-8'>\n"
"  <meta name='viewport' content='width=device-width, initial-scale=1'>\n"
"  <title>ESP32 Step Motor</title>\n"
"  <style>\n"
"    body { font-family: sans-serif; margin: 0; background: #111827; color: #e5e7eb; }\n"
"    header { padding: 16px; background: #0f172a; border-bottom: 1px solid #334155; }\n"
"    main { padding: 16px; max-width: 1000px; margin: 0 auto; display: grid; gap: 16px; }\n"
"    .card { background: #1f2937; border: 1px solid #374151; border-radius: 14px; padding: 16px; }\n"
"    .grid { display: grid; gap: 12px; grid-template-columns: repeat(auto-fit,minmax(180px,1fr)); }\n"
"    label { display: grid; gap: 6px; font-size: 14px; }\n"
"    input, select, button { background: #0b1220; color: #e5e7eb; border: 1px solid #475569; border-radius: 10px; padding: 10px 12px; font-size: 14px; }\n"
"    button { cursor: pointer; font-weight: 600; }\n"
"    button.primary { background: #2563eb; border-color: #2563eb; }\n"
"    button.good { background: #16a34a; border-color: #16a34a; }\n"
"    button.warn { background: #dc2626; border-color: #dc2626; }\n"
"    .row { display: flex; flex-wrap: wrap; gap: 10px; }\n"
"    .pill { display: inline-block; padding: 4px 10px; border-radius: 999px; background: #334155; margin-right: 6px; margin-bottom: 6px; }\n"
"    .muted { color: #94a3b8; }\n"
"    pre { white-space: pre-wrap; word-break: break-word; margin: 0; }\n"
"    code { background: #0b1220; padding: 2px 6px; border-radius: 6px; }\n"
"    details { margin-top: 10px; }\n"
"    summary { cursor: pointer; font-weight: 600; }\n"
"    ol { margin-top: 0.5rem; }\n"
"    progress { width: 100%; height: 18px; }\n"
"  </style>\n"
"</head>\n"
"<body>\n"
"  <header>\n"
"    <h1>ESP32 Step Motor</h1>\n"
"    <div class='muted'>HTTP API + local control page</div>\n"
"  </header>\n"
"  <main>\n"
"    <section class='card'>\n"
"      <h2>Motion config</h2>\n"
"      <div class='grid'>\n"
"        <label>Profile\n"
"          <select id='profile'>\n"
"            <option value='time'>time</option>\n"
"            <option value='steps'>steps</option>\n"
"          </select>\n"
"        </label>\n"
"        <label>Step period (us)\n"
"          <input id='step_period_us' type='number' min='50' step='1' value='1000'>\n"
"        </label>\n"
"        <label>Move time (ms)\n"
"          <input id='move_time_ms' type='number' min='0' step='1' value='5000'>\n"
"        </label>\n"
"        <label>Steps per move\n"
"          <input id='move_steps' type='number' min='0' step='1' value='200'>\n"
"        </label>\n"
"        <label>Pause (ms)\n"
"          <input id='pause_ms' type='number' min='0' step='1' value='1000'>\n"
"        </label>\n"
"        <label>Dir setup (us)\n"
"          <input id='dir_setup_us' type='number' min='0' step='1' value='20'>\n"
"        </label>\n"
"      </div>\n"
"      <div class='row' style='margin-top:12px'>\n"
"        <button class='primary' onclick='saveMotionConfig()'>Apply settings</button>\n"
"        <button onclick='refreshState()'>Refresh</button>\n"
"      </div>\n"
"    </section>\n"
"\n"
"    <section class='card'>\n"
"      <h2>Manual control</h2>\n"
"      <div class='row'>\n"
"        <button class='good' onclick='sendAction(\"forward\")'>Jog forward</button>\n"
"        <button class='good' onclick='sendAction(\"reverse\")'>Jog reverse</button>\n"
"        <button class='warn' onclick='sendAction(\"stop\")'>Stop</button>\n"
"        <button class='primary' onclick='sendAction(\"auto_start\")'>Start auto</button>\n"
"        <button class='warn' onclick='sendAction(\"auto_stop\")'>Stop auto</button>\n"
"      </div>\n"
"    </section>\n"
"\n"
"    <section class='card'>\n"
"      <h2>Wi-Fi setup</h2>\n"
"      <p class='muted'>O AP de emergência permanece disponível em <code>192.168.4.1</code> para recuperação e configuração, mesmo quando o modo principal for a rede do roteador.</p>\n"
"      <div class='grid'>\n"
"        <label>Modo principal\n"
"          <select id='wifi_mode'>\n"
"            <option value='ap'>AP local / emergência</option>\n"
"            <option value='sta'>Wi-Fi do roteador</option>\n"
"          </select>\n"
"        </label>\n"
"        <label>SSID do roteador\n"
"          <input id='wifi_ssid' type='text' maxlength='32' placeholder='Minha rede Wi-Fi'>\n"
"        </label>\n"
"        <label>Senha do roteador\n"
"          <input id='wifi_password' type='password' maxlength='64' placeholder='Opcional'>\n"
"        </label>\n"
"      </div>\n"
"      <div class='row' style='margin-top:12px'>\n"
"        <button class='primary' onclick='saveWifiConfig()'>Salvar Wi-Fi</button>\n"
"        <button onclick='refreshState()'>Atualizar</button>\n"
"      </div>\n"
"    </section>\n"
"\n"
"    <section class='card'>\n"
"      <h2>Como usar</h2>\n"
"      <ol>\n"
"        <li>Abra a página no <code>http://192.168.4.1/</code> ou no IP da sua LAN quando o modo cliente estiver ativo.</li>\n"
"        <li>Escolha o modo principal: <strong>AP local</strong> ou <strong>Wi-Fi do roteador</strong>.</li>\n"
"        <li>Se for usar a rede do roteador, informe SSID e senha e clique em <strong>Salvar Wi-Fi</strong>.</li>\n"
"        <li>Use os botões manuais para testar o motor ou deixe o modo automático ligado.</li>\n"
"        <li>Para integrações, use os endpoints listados abaixo.</li>\n"
"      </ol>\n"
"      <details>\n"
"        <summary>Integração com Home Assistant</summary>\n"
"        <p>Exemplo simples com <code>rest_command</code>:</p>\n"
"        <pre>rest_command:\n"
"  step_motor_forward:\n"
"    url: http://192.168.4.1/api/control\n"
"    method: POST\n"
"    content_type: application/json\n"
"    payload: '{\"action\":\"forward\"}'\n"
"\n"
"  step_motor_reverse:\n"
"    url: http://192.168.4.1/api/control\n"
"    method: POST\n"
"    content_type: application/json\n"
"    payload: '{\"action\":\"reverse\"}'\n"
"\n"
"  step_motor_stop:\n"
"    url: http://192.168.4.1/api/control\n"
"    method: POST\n"
"    content_type: application/json\n"
"    payload: '{\"action\":\"stop\"}'\n"
"\n"
"  step_motor_auto_start:\n"
"    url: http://192.168.4.1/api/control\n"
"    method: POST\n"
"    content_type: application/json\n"
"    payload: '{\"action\":\"auto_start\"}'\n"
"\n"
"  step_motor_auto_stop:\n"
"    url: http://192.168.4.1/api/control\n"
"    method: POST\n"
"    content_type: application/json\n"
"    payload: '{\"action\":\"auto_stop\"}'</pre>\n"
"      </details>\n"
"      <details>\n"
"        <summary>Endpoints disponíveis</summary>\n"
"        <ul>\n"
"          <li><code>GET /api/health</code></li>\n"
"          <li><code>GET /api/state</code></li>\n"
"          <li><code>POST /api/config</code></li>\n"
"          <li><code>POST /api/control</code></li>\n"
"          <li><code>GET /api/wifi</code></li>\n"
"          <li><code>POST /api/wifi</code></li>\n"
"          <li><code>GET /api/ota</code></li>\n"
"          <li><code>POST /api/ota</code></li>\n"
"        </ul>\n"
"      </details>\n"
"    </section>\n"
"\n"
"    <section class='card'>\n"
"      <h2>Firmware update (OTA)</h2>\n"
"      <p class='muted'>Upload a firmware .bin file. The device writes it to the alternate OTA slot, marks it for boot, and reboots automatically. The emergency AP stays available.</p>\n"
"      <div class='grid'>\n"
"        <label>Firmware image\n"
"          <input id='ota_file' type='file' accept='.bin,application/octet-stream' onchange='onOtaFileSelected()'>\n"
"        </label>\n"
"      </div>\n"
"      <div id='ota_file_info' class='muted' style='margin-top:8px'>No firmware image selected.</div>\n"
"      <div class='row' style='margin-top:12px'>\n"
"        <button id='ota_upload_btn' class='primary' onclick='uploadOta()'>Upload and reboot</button>\n"
"        <button id='ota_refresh_btn' onclick='refreshOtaStatus()'>Refresh OTA status</button>\n"
"      </div>\n"
"      <div id='ota_status' class='muted' style='margin-top:8px'>Ready.</div>\n"
"      <progress id='ota_progress' value='0' max='100' style='display:none;'></progress>\n"
"    </section>\n"
"\n"
"    <section class='card'>\n"
"      <h2>Status</h2>\n"
"      <div id='status'>Loading...</div>\n"
"      <pre id='json' class='muted'></pre>\n"
"    </section>\n"
"  </main>\n"
"  <script>\n"
"    async function api(path, body) {\n"
"      const opts = body ? { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body) } : {};\n"
"      const res = await fetch(path, opts);\n"
"      if (!res.ok) throw new Error(await res.text());\n"
"      return res.json();\n"
"    }\n"
"\n"
"    const dirtyFields = new Set();\n"
"\n"
"    function markFieldDirty(el) {\n"
"      if (el && el.id) {\n"
"        dirtyFields.add(el.id);\n"
"      }\n"
"    }\n"
"\n"
"    function clearDirtyFields() {\n"
"      dirtyFields.clear();\n"
"    }\n"
"\n"
"    function syncField(id, value) {\n"
"      const el = document.getElementById(id);\n"
"      if (!el) return;\n"
"      if (dirtyFields.has(id)) return;\n"
"      if (document.activeElement === el) return;\n"
"      const next = String(value);\n"
"      if (el.value !== next) {\n"
"        el.value = next;\n"
"      }\n"
"    }\n"
"\n"
"    async function refreshState() {\n"
"      const state = await api('/api/state');\n"
"      document.getElementById('json').textContent = JSON.stringify(state, null, 2);\n"
"      syncField('profile', state.profile);\n"
"      syncField('step_period_us', state.step_period_us);\n"
"      syncField('move_time_ms', state.move_time_ms);\n"
"      syncField('move_steps', state.move_steps);\n"
"      syncField('pause_ms', state.pause_ms);\n"
"      syncField('dir_setup_us', state.dir_setup_us);\n"
"      syncField('wifi_mode', state.wifi_mode);\n"
"      syncField('wifi_ssid', state.wifi_ssid);\n"
"\n"
"      const wifi = state.wifi_sta_connected ? ('STA ' + state.wifi_sta_ip) : 'STA desconectado';\n"
"      const ap = state.wifi_ap_started ? ('AP ' + state.wifi_ap_ip) : 'AP off';\n"
"      document.getElementById('status').innerHTML = `\n"
"        <span class='pill'>${state.running ? 'RUNNING' : 'IDLE'}</span>\n"
"        <span class='pill'>${state.auto_mode ? 'AUTO' : 'MANUAL'}</span>\n"
"        <span class='pill'>profile ${state.profile}</span>\n"
"        <span class='pill'>mode ${state.wifi_mode}</span>\n"
"        <span class='pill'>${wifi}</span>\n"
"        <span class='pill'>${ap}</span>\n"
"        <span class='pill'>SSID ${state.wifi_ssid || '—'}</span>\n"
"        <span class='pill'>freq ${state.step_frequency_hz} Hz</span>\n"
"        <span class='pill'>move ${state.move_duration_ms} ms</span>\n"
"        <span class='pill'>emergency AP ${state.wifi_emergency_ap ? 'on' : 'off'}</span>\n"
"        <span class='pill'>pending ${state.pending_action}</span>\n"
"      `;\n"
"      refreshOtaStatus().catch(console.error);\n"
"    }\n"
"\n"
"    async function refreshOtaStatus() {\n"
"      const state = await api('/api/ota');\n"
"      const parts = [];\n"
"      parts.push(`version ${state.firmware_version}`);\n"
"      parts.push(`running ${state.running_partition}`);\n"
"      parts.push(`update ${state.next_partition}`);\n"
"      parts.push(state.pending_verify ? 'pending verify' : 'confirmed');\n"
"      parts.push(state.ota_supported ? `slot ${state.slot_size} bytes` : 'OTA disabled');\n"
"      document.getElementById('ota_status').textContent = parts.join(' • ');\n"
"    }\n"
"\n"
"    function onOtaFileSelected() {\n"
"      const input = document.getElementById('ota_file');\n"
"      const info = document.getElementById('ota_file_info');\n"
"      const file = input.files && input.files[0];\n"
"      if (!file) {\n"
"        info.textContent = 'No firmware image selected.';\n"
"        return;\n"
"      }\n"
"      const sizeKb = Math.round(file.size / 1024);\n"
"      info.textContent = `Selected: ${file.name} (${sizeKb} KB)`;\n"
"    }\n"
"\n"
"    function setOtaControlsDisabled(disabled) {\n"
"      const ids = ['ota_file', 'ota_upload_btn', 'ota_refresh_btn'];\n"
"      ids.forEach((id) => {\n"
"        const el = document.getElementById(id);\n"
"        if (el) {\n"
"          el.disabled = disabled;\n"
"        }\n"
"      });\n"
"    }\n"
"\n"
"    function waitForOtaReconnect(attempt = 0) {\n"
"      const status = document.getElementById('ota_status');\n"
"      if (attempt === 0) {\n"
"        status.textContent = 'Rebooting the device... waiting for it to come back online.';\n"
"      }\n"
"\n"
"      setTimeout(() => {\n"
"        refreshState()\n"
"          .then(() => {\n"
"            status.textContent = 'Device is back online after OTA reboot.';\n"
"            setOtaControlsDisabled(false);\n"
"            const info = document.getElementById('ota_file_info');\n"
"            if (info) {\n"
"              info.textContent = 'Firmware upload complete. You can select a new file for the next update.';\n"
"            }\n"
"          })\n"
"          .catch(() => {\n"
"            if (attempt < 30) {\n"
"              waitForOtaReconnect(attempt + 1);\n"
"            } else {\n"
"              status.textContent = 'Upload finished, but the device did not come back online automatically. Refresh manually in a few seconds.';\n"
"              setOtaControlsDisabled(false);\n"
"            }\n"
"          });\n"
"      }, attempt === 0 ? 1500 : 2000);\n"
"    }\n"
"\n"
"    async function uploadOta() {\n"
"      const input = document.getElementById('ota_file');\n"
"      const file = input.files && input.files[0];\n"
"      const status = document.getElementById('ota_status');\n"
"      const progress = document.getElementById('ota_progress');\n"
"      if (!file) {\n"
"        status.textContent = 'Select a .bin firmware image first.';\n"
"        return;\n"
"      }\n"
"\n"
"      setOtaControlsDisabled(true);\n"
"      progress.style.display = 'block';\n"
"      progress.value = 0;\n"
"      status.textContent = 'Uploading firmware...';\n"
"\n"
"      await new Promise((resolve, reject) => {\n"
"        const xhr = new XMLHttpRequest();\n"
"        xhr.open('POST', '/api/ota');\n"
"        xhr.responseType = 'json';\n"
"        xhr.setRequestHeader('Content-Type', 'application/octet-stream');\n"
"        xhr.upload.onprogress = (event) => {\n"
"          if (event.lengthComputable) {\n"
"            progress.value = Math.round((event.loaded / event.total) * 100);\n"
"            status.textContent = `Uploading firmware... ${progress.value}%`;\n"
"          }\n"
"        };\n"
"        xhr.onload = () => {\n"
"          progress.style.display = 'none';\n"
"          if (xhr.status >= 200 && xhr.status < 300) {\n"
"            const rebootMs = (xhr.response && xhr.response.rebooting_in_ms) || 1500;\n"
"            status.textContent = `Upload complete. Rebooting in ${Math.round(rebootMs / 100) / 10}s...`;\n"
"            input.value = '';\n"
"            const info = document.getElementById('ota_file_info');\n"
"            if (info) {\n"
"              info.textContent = 'Firmware uploaded successfully. Waiting for reboot...';\n"
"            }\n"
"            waitForOtaReconnect();\n"
"            resolve();\n"
"          } else {\n"
"            setOtaControlsDisabled(false);\n"
"            const message = (xhr.response && xhr.response.message) || xhr.responseText || `HTTP ${xhr.status}`;\n"
"            status.textContent = `OTA failed: ${message}`;\n"
"            reject(new Error(message));\n"
"          }\n"
"        };\n"
"        xhr.onerror = () => {\n"
"          progress.style.display = 'none';\n"
"          setOtaControlsDisabled(false);\n"
"          status.textContent = 'OTA upload failed due to a network error.';\n"
"          reject(new Error('network error'));\n"
"        };\n"
"        xhr.send(file);\n"
"      });\n"
"    }\n"
"\n"
"    async function saveMotionConfig() {\n"
"      await api('/api/config', {\n"
"        profile: document.getElementById('profile').value,\n"
"        step_period_us: parseInt(document.getElementById('step_period_us').value, 10),\n"
"        move_time_ms: parseInt(document.getElementById('move_time_ms').value, 10),\n"
"        move_steps: parseInt(document.getElementById('move_steps').value, 10),\n"
"        pause_ms: parseInt(document.getElementById('pause_ms').value, 10),\n"
"        dir_setup_us: parseInt(document.getElementById('dir_setup_us').value, 10)\n"
"      });\n"
"      clearDirtyFields();\n"
"      await refreshState();\n"
"    }\n"
"\n"
"    async function saveWifiConfig() {\n"
"      await api('/api/wifi', {\n"
"        mode: document.getElementById('wifi_mode').value,\n"
"        ssid: document.getElementById('wifi_ssid').value,\n"
"        password: document.getElementById('wifi_password').value\n"
"      });\n"
"      document.getElementById('wifi_password').value = '';\n"
"      clearDirtyFields();\n"
"      await refreshState();\n"
"    }\n"
"\n"
"    async function sendAction(action) {\n"
"      await api('/api/control', { action });\n"
"      await refreshState();\n"
"    }\n"
"\n"
"    document.querySelectorAll('input, select').forEach((el) => {\n"
"      el.addEventListener('input', () => markFieldDirty(el));\n"
"      el.addEventListener('change', () => markFieldDirty(el));\n"
"    });\n"
"\n"
"    refreshState().catch(console.error);\n"
"    setInterval(() => refreshState().catch(console.error), 1000);\n"
"  </script>\n"
"</body>\n"
"</html>\n"
;

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
    g_state.profile = MOTION_PROFILE_TIME;
    g_state.step_period_us = STEP_PERIOD_US;
    g_state.move_time_ms = MOVE_TIME_MS;
    g_state.move_steps = STEP_COUNT;
    g_state.pause_ms = PAUSE_MS;
    g_state.dir_setup_us = DIR_SETUP_US;
    g_state.auto_mode = false;
    g_state.running = false;
    g_state.activity = MOTOR_ACTIVITY_IDLE;
    g_state.pending_action = MOTOR_ACTION_NONE;
    g_state.wifi_sta_connected = false;
    g_state.wifi_ap_started = false;
    g_state.wifi_sta_ip[0] = '\0';
    g_state.wifi_ap_ip[0] = '\0';
    g_state.wifi_mode[0] = '\0';
    g_state.wifi_ssid[0] = '\0';
    g_state.wifi_emergency_ap = true;
    snprintf(g_state.hostname, sizeof(g_state.hostname), "%s", HOSTNAME);
    wifi_settings_apply_to_state();
}

static app_state_t state_snapshot(void)
{
    app_state_t snapshot;
    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    snapshot = g_state;
    xSemaphoreGive(g_state_mutex);
    return snapshot;
}

static void notify_motor_task(void)
{
    if (g_motor_task_handle != NULL) {
        xTaskNotifyGive(g_motor_task_handle);
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

static void motor_enable(bool enable)
{
    if (EN_ACTIVE_LOW) {
        gpio_set_level(EN_GPIO, enable ? 0 : 1);
    } else {
        gpio_set_level(EN_GPIO, enable ? 1 : 0);
    }
}

static void motor_stop_output(void)
{
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_SPEED_MODE, LEDC_CHANNEL_NUM, 0));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_SPEED_MODE, LEDC_CHANNEL_NUM));
}

static void motor_apply_pwm_period(uint32_t step_period_us)
{
    step_period_us = clamp_u32(step_period_us, 50U, 1000000U);
    if (step_period_us == g_current_step_period_us && g_pwm_resolution_bits != 0U) {
        return;
    }

    const uint32_t freq_hz = step_frequency_hz_from_period(step_period_us);
    const uint32_t resolution_bits = ledc_resolution_bits_for_frequency(freq_hz);

    ledc_timer_config_t timer = {
        .speed_mode = LEDC_SPEED_MODE,
        .duty_resolution = (ledc_timer_bit_t)resolution_bits,
        .timer_num = LEDC_TIMER_NUM,
        .freq_hz = freq_hz,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));

    g_current_step_period_us = step_period_us;
    g_pwm_resolution_bits = resolution_bits;
}

static void motor_step_output_start(void)
{
    const uint32_t max_duty = (1U << g_pwm_resolution_bits) - 1U;
    const uint32_t duty = max_duty / 2U;
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_SPEED_MODE, LEDC_CHANNEL_NUM, duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_SPEED_MODE, LEDC_CHANNEL_NUM));
}

static motor_action_t take_pending_action(void)
{
    motor_action_t action;
    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    action = g_state.pending_action;
    g_state.pending_action = MOTOR_ACTION_NONE;
    xSemaphoreGive(g_state_mutex);
    return action;
}

static motor_action_t peek_pending_action(void)
{
    motor_action_t action;
    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    action = g_state.pending_action;
    xSemaphoreGive(g_state_mutex);
    return action;
}

static uint64_t motion_duration_us_from_state(const app_state_t *state)
{
    if (state->profile == MOTION_PROFILE_STEPS) {
        return (uint64_t)state->move_steps * (uint64_t)state->step_period_us;
    }
    return (uint64_t)state->move_time_ms * 1000ULL;
}

static bool wait_abortable_us(uint64_t duration_us)
{
    const uint64_t slice_us = 20000ULL;
    const int64_t end_time = esp_timer_get_time() + (int64_t)duration_us;

    while (true) {
        if (peek_pending_action() != MOTOR_ACTION_NONE) {
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

static bool motor_run_segment(bool forward, uint64_t duration_us, uint32_t dir_setup_us)
{
    if (duration_us == 0U) {
        return true;
    }

    gpio_set_level(DIR_GPIO, forward ? 1 : 0);
    esp_rom_delay_us(dir_setup_us);

    motor_step_output_start();
    const bool completed = wait_abortable_us(duration_us);
    motor_stop_output();
    return completed;
}

static bool motion_wait_pause(uint32_t pause_ms)
{
    return wait_abortable_us((uint64_t)pause_ms * 1000ULL);
}

static void set_activity_idle(app_state_t *state)
{
    state->activity = MOTOR_ACTIVITY_IDLE;
    state->running = false;
    state->auto_mode = false;
}

static void set_activity_running(app_state_t *state)
{
    state->activity = MOTOR_ACTIVITY_RUNNING;
    state->running = true;
    state->auto_mode = false;
}

static void set_activity_auto(app_state_t *state)
{
    state->activity = MOTOR_ACTIVITY_AUTO;
    state->running = true;
    state->auto_mode = true;
}

static void motor_task(void *arg)
{
    (void)arg;

    while (true) {
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100));

        motor_action_t action = take_pending_action();
        if (action == MOTOR_ACTION_NONE) {
            const app_state_t snapshot = state_snapshot();
            if (snapshot.auto_mode) {
                action = MOTOR_ACTION_START_AUTO;
            } else {
                continue;
            }
        }

        if (action == MOTOR_ACTION_STOP) {
            motor_stop_output();
            xSemaphoreTake(g_state_mutex, portMAX_DELAY);
            set_activity_idle(&g_state);
            xSemaphoreGive(g_state_mutex);
            continue;
        }

        if (action == MOTOR_ACTION_JOG_FORWARD || action == MOTOR_ACTION_JOG_REVERSE) {
            const app_state_t snapshot = state_snapshot();
            motor_apply_pwm_period(snapshot.step_period_us);

            xSemaphoreTake(g_state_mutex, portMAX_DELAY);
            set_activity_running(&g_state);
            xSemaphoreGive(g_state_mutex);

            const uint64_t duration_us = motion_duration_us_from_state(&snapshot);
            (void)motor_run_segment(action == MOTOR_ACTION_JOG_FORWARD, duration_us, snapshot.dir_setup_us);

            motor_stop_output();
            xSemaphoreTake(g_state_mutex, portMAX_DELAY);
            set_activity_idle(&g_state);
            xSemaphoreGive(g_state_mutex);
            continue;
        }

        if (action == MOTOR_ACTION_START_AUTO) {
            while (true) {
                app_state_t snapshot = state_snapshot();
                if (!snapshot.auto_mode) {
                    break;
                }

                motor_apply_pwm_period(snapshot.step_period_us);

                xSemaphoreTake(g_state_mutex, portMAX_DELAY);
                set_activity_auto(&g_state);
                xSemaphoreGive(g_state_mutex);

                const uint64_t duration_us = motion_duration_us_from_state(&snapshot);
                if (!motor_run_segment(true, duration_us, snapshot.dir_setup_us)) {
                    break;
                }

                if (!motion_wait_pause(snapshot.pause_ms)) {
                    break;
                }

                snapshot = state_snapshot();
                if (!snapshot.auto_mode) {
                    break;
                }

                motor_apply_pwm_period(snapshot.step_period_us);
                if (!motor_run_segment(false, duration_us, snapshot.dir_setup_us)) {
                    break;
                }

                if (!motion_wait_pause(snapshot.pause_ms)) {
                    break;
                }
            }

            motor_stop_output();
            xSemaphoreTake(g_state_mutex, portMAX_DELAY);
            set_activity_idle(&g_state);
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

static void build_state_json(char *buf, size_t len)
{
    const app_state_t s = state_snapshot();
    const uint64_t duration_us = motion_duration_us_from_state(&s);
    const uint32_t move_duration_ms = (uint32_t)((duration_us + 999ULL) / 1000ULL);
    const uint32_t freq_hz = step_frequency_hz_from_period(s.step_period_us);
    const esp_app_desc_t *app_desc = esp_app_get_description();
    const esp_partition_t *running_partition = esp_ota_get_running_partition();
    esp_ota_img_states_t ota_state;
    const bool ota_pending_verify = (running_partition != NULL && esp_ota_get_state_partition(running_partition, &ota_state) == ESP_OK && ota_state == ESP_OTA_IMG_PENDING_VERIFY);
    const char *firmware_version = (app_desc != NULL) ? app_desc->version : "unknown";
    const char *running_partition_label = partition_label_or_unknown(running_partition);

    snprintf(buf, len,
             "{"
             "\"profile\":\"%s\","
             "\"auto_mode\":%s,"
             "\"running\":%s,"
             "\"activity\":\"%s\","
             "\"pending_action\":\"%s\","
             "\"step_period_us\":%" PRIu32 ","
             "\"step_frequency_hz\":%" PRIu32 ","
             "\"move_time_ms\":%" PRIu32 ","
             "\"move_steps\":%" PRIu32 ","
             "\"move_duration_ms\":%" PRIu32 ","
             "\"pause_ms\":%" PRIu32 ","
             "\"dir_setup_us\":%" PRIu32 ","
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
             motion_profile_to_string(s.profile),
             bool_to_json(s.auto_mode),
             bool_to_json(s.running),
             motor_activity_to_string(s.activity),
             motor_action_to_string(s.pending_action),
             s.step_period_us,
             freq_hz,
             s.move_time_ms,
             s.move_steps,
             move_duration_ms,
             s.pause_ms,
             s.dir_setup_us,
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
    char json[1024];
    build_state_json(json, sizeof(json));
    return send_json(req, json);
}

static esp_err_t config_post_handler(httpd_req_t *req)
{
    char *body = read_request_body(req);
    if (body == NULL) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid request body");
    }

    app_state_t snapshot = state_snapshot();
    uint32_t value;
    char str[64];

    if (json_get_string_field(body, "profile", str, sizeof(str))) {
        snapshot.profile = (strcmp(str, "steps") == 0) ? MOTION_PROFILE_STEPS : MOTION_PROFILE_TIME;
    }

    if (json_get_uint32_field(body, "step_period_us", &value)) {
        snapshot.step_period_us = clamp_u32(value, 50U, 1000000U);
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
    g_state.profile = snapshot.profile;
    g_state.step_period_us = snapshot.step_period_us;
    g_state.move_time_ms = snapshot.move_time_ms;
    g_state.move_steps = snapshot.move_steps;
    g_state.pause_ms = snapshot.pause_ms;
    g_state.dir_setup_us = snapshot.dir_setup_us;
    xSemaphoreGive(g_state_mutex);

    free(body);
    notify_motor_task();

    char json[1024];
    build_state_json(json, sizeof(json));
    return send_json(req, json);
}

static esp_err_t control_post_handler(httpd_req_t *req)
{
    char *body = read_request_body(req);
    if (body == NULL) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid request body");
    }

    char action[32];
    if (!json_get_string_field(body, "action", action, sizeof(action))) {
        free(body);
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "missing action");
    }

    motor_action_t pending = MOTOR_ACTION_NONE;
    bool auto_mode = false;

    if (strcmp(action, "forward") == 0) {
        pending = MOTOR_ACTION_JOG_FORWARD;
    } else if (strcmp(action, "reverse") == 0) {
        pending = MOTOR_ACTION_JOG_REVERSE;
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
    g_state.pending_action = pending;
    g_state.auto_mode = auto_mode;
    if (pending == MOTOR_ACTION_STOP) {
        g_state.auto_mode = false;
    }
    xSemaphoreGive(g_state_mutex);

    free(body);
    notify_motor_task();

    char json[1024];
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

    motor_apply_pwm_period(g_state.step_period_us);
    motor_stop_output();

    ESP_LOGI(TAG, "Pins STEP=%d DIR=%d EN=%d", STEP_GPIO, DIR_GPIO, EN_GPIO);
    ESP_LOGI(TAG, "Enable pin is %s", EN_ACTIVE_LOW ? "active-low" : "active-high");
    ESP_LOGI(TAG, "Defaults: profile=%s step_period=%" PRIu32 " us move_time=%" PRIu32 " ms move_steps=%" PRIu32 " pause=%" PRIu32 " ms",
             motion_profile_to_string(g_state.profile), g_state.step_period_us, g_state.move_time_ms, g_state.move_steps, g_state.pause_ms);
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

    xTaskCreate(motor_task, "motor_task", 4096, NULL, 5, &g_motor_task_handle);
    motor_enable(true);

    ESP_LOGI(TAG, "HTTP server ready: http://%s/", HOSTNAME);
    ESP_LOGI(TAG, "API endpoints: GET /api/health, GET /api/state, POST /api/config, POST /api/control, GET /api/wifi, POST /api/wifi, GET /api/ota, POST /api/ota");
}
