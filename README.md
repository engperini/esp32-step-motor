# esp32-step-motor

Projeto ESP-IDF para testar um driver TMC2208 com um XIAO ESP32-S3 Sense.

## O que ele faz

- habilita o driver
- define direção via `DIR`
- gera pulsos de STEP com *hardware PWM* no ESP32
- anda por um tempo fixo para frente e depois o mesmo tempo para trás
- pausa entre as trocas de direção

## Por que este firmware é melhor

A geração de pulsos não é feita em loop apertado de software.

O `STEP` sai por periférico de hardware, então:

- não trava a CPU
- não dispara watchdog por busy-wait
- mantém frequência estável
- é mais apropriado para teste real de motor de passo

## Ligações

Edite os pinos em `main/main.c`:

- `STEP_GPIO`
- `DIR_GPIO`
- `EN_GPIO`

> O enable do TMC2208 é normalmente *ativo em nível baixo*.

## Parâmetros principais

No `main/main.c` você pode mudar:

- `STEP_PERIOD_US` — período entre pulsos de STEP
- `MOVE_TIME_MS` — tempo andando em cada direção
- `PAUSE_MS` — pausa entre frente e ré
- `DIR_SETUP_US` — tempo de setup da direção antes de iniciar os pulsos

## Build no Ubuntu / ESP-IDF

```bash
cd /home/pi/esp32-step-motor
source /root/esp-idf/export.sh
idf.py set-target esp32s3
idf.py build
```

## Flash

Exemplo:

```bash
idf.py -p /tmp/ttyesp32 flash monitor
```

Se a sua porta for outra, troque pelo device correto.

## Observação

Este projeto continua simples de propósito, mas agora a parte crítica do STEP usa hardware PWM.
