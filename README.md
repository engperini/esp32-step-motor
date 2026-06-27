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

A placa sobe um *fallback AP* sempre:

- SSID padrão: `esp32-step-motor`
- senha padrão: `stepmotor123`

Para conectar o ESP32 à sua rede e usar Home Assistant na LAN, abra `idf.py menuconfig` e preencha:

- *Step Motor Wi-Fi Settings → Wi-Fi SSID*
- *Step Motor Wi-Fi Settings → Wi-Fi password*

Se a STA conectar, use o IP mostrado na tela ou nos logs do boot.

O AP fallback sempre sobe em:

- `http://192.168.4.1/`

## Página web

Abra no navegador:

- `http://192.168.4.1/` quando estiver no AP fallback
- ou o IP da sua rede local mostrado na tela / logs quando a STA conectar

Na página você consegue:

- escolher `time` ou `steps`
- ajustar `step_period_us`
- ajustar `move_time_ms`
- ajustar `move_steps`
- ajustar `pause_ms`
- ajustar `dir_setup_us`
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

### `GET /api/state`

Retorna JSON com o estado atual, por exemplo:

```json
{
  "profile": "time",
  "auto_mode": false,
  "running": false,
  "activity": "idle",
  "pending_action": "none"
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

  step_motor_auto_start:
    url: http://192.168.4.1/api/control
    method: POST
    content_type: application/json
    payload: '{"action":"auto_start"}'
```

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
