# esp32-step-motor

Firmware ESP-IDF para um TMC2208 com XIAO ESP32-S3 Sense, agora com:

- controle do motor via *hardware PWM* no STEP
- servidor HTTP embutido
- API JSON para integração com Home Assistant
- página web local para controle manual e configuração
- modo automático alternando frente / ré
- modo por *tempo* ou por *quantidade de passos*

## Pinos padrão

Edite em `main/main.c` se quiser mudar a pinagem:

- `STEP_GPIO = GPIO_NUM_4`
- `DIR_GPIO = GPIO_NUM_5`
- `EN_GPIO = GPIO_NUM_6`

O enable do TMC2208 é *ativo em nível baixo* por padrão.

## Wi-Fi

O firmware agora trabalha com **AP de emergência + configuração em tempo de execução**.

### Modos disponíveis na página

- **AP local / emergência**
  - mantém o acesso em `http://192.168.4.1/`
  - ideal para setup, manutenção e uso sem roteador
- **Wi-Fi do roteador**
  - o ESP32 tenta entrar na sua rede doméstica/industrial
  - o AP de emergência continua disponível para recuperação

As credenciais e o modo ficam salvos em NVS e podem ser alterados a qualquer momento pela interface web.

## Página web

Abra no navegador:

- `http://192.168.4.1/` quando estiver no AP de emergência
- ou o IP da sua rede local quando o modo cliente estiver ativo

Na página você consegue:

- escolher `time` ou `steps`
- ajustar `step_period_us`
- ajustar `move_time_ms`
- ajustar `move_steps`
- ajustar `pause_ms`
- ajustar `dir_setup_us`
- configurar o modo Wi-Fi
- salvar SSID e senha do roteador
- mandar `Jog forward`
- mandar `Jog reverse`
- `Stop`
- `Start auto`
- `Stop auto`

## API HTTP

Endpoints disponíveis:

- `GET /api/health`
- `GET /api/state`
- `POST /api/config`
- `POST /api/control`
- `GET /api/wifi`
- `POST /api/wifi`

### `GET /api/state`

Retorna JSON com o estado do motor e do Wi-Fi. Exemplo resumido:

```json
{
  "profile": "time",
  "auto_mode": false,
  "running": false,
  "activity": "idle",
  "pending_action": "none",
  "wifi_mode": "ap",
  "wifi_ssid": "MinhaRede",
  "wifi_sta_connected": false,
  "wifi_ap_ip": "192.168.4.1"
}
```

### `POST /api/config`

Exemplo de body:

```json
{
  "profile": "steps",
  "step_period_us": 1000,
  "move_time_ms": 5000,
  "move_steps": 200,
  "pause_ms": 1000,
  "dir_setup_us": 20
}
```

### `GET /api/wifi`

Retorna o estado/configuração de Wi-Fi:

```json
{
  "mode": "sta",
  "ssid": "MinhaRede",
  "sta_connected": true,
  "sta_ip": "192.168.1.50",
  "ap_started": true,
  "ap_ip": "192.168.4.1",
  "emergency_ap": true
}
```

### `POST /api/wifi`

Exemplo de body:

```json
{
  "mode": "sta",
  "ssid": "MinhaRede",
  "password": "minha_senha"
}
```

- `mode`: `ap` ou `sta`
- `ssid` e `password` são opcionais ao mudar apenas o modo
- o AP de emergência permanece ativo para recuperação

### `POST /api/control`

Exemplos de `action`:

- `forward`
- `reverse`
- `stop`
- `auto_start`
- `auto_stop`

## Home Assistant

Você pode integrar com `rest_command`, `rest` sensor, botões ou scripts HTTP.

Exemplo simples de `rest_command`:

```yaml
rest_command:
  step_motor_forward:
    url: http://192.168.4.1/api/control
    method: POST
    content_type: application/json
    payload: '{"action":"forward"}'

  step_motor_reverse:
    url: http://192.168.4.1/api/control
    method: POST
    content_type: application/json
    payload: '{"action":"reverse"}'

  step_motor_stop:
    url: http://192.168.4.1/api/control
    method: POST
    content_type: application/json
    payload: '{"action":"stop"}'

  step_motor_auto_start:
    url: http://192.168.4.1/api/control
    method: POST
    content_type: application/json
    payload: '{"action":"auto_start"}'

  step_motor_auto_stop:
    url: http://192.168.4.1/api/control
    method: POST
    content_type: application/json
    payload: '{"action":"auto_stop"}'
```

Se o ESP32 estiver conectado ao roteador, substitua `192.168.4.1` pelo IP da LAN.

## Observação

A geração do STEP fica no periférico de hardware, então o firmware não depende de loops pesados de software para manter o watchdog feliz.

## Build

```bash
cd /home/pi/esp32-step-motor
source /root/esp-idf/export.sh
idf.py set-target esp32s3
idf.py build
```

## Flash

```bash
idf.py -p /tmp/ttyesp32 flash monitor
```

## Observação

Este projeto foi feito para ser simples de integrar e estável para teste real. A geração do STEP fica no periférico de hardware, então o firmware não depende mais de loops pesados de software para manter o watchdog feliz.
