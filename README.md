# esp32-step-motor

Projeto ESP-IDF mínimo para testar um driver TMC2208 com um XIAO ESP32-S3 Sense.

Ele faz só o básico:
- habilita o driver
- define direção
- gera pulsos no STEP
- anda por um tempo fixo para frente e depois o mesmo tempo para trás

## Ligações

Edite os pinos em `main/main.c`:

- `STEP_GPIO`
- `DIR_GPIO`
- `EN_GPIO`

> O enable do TMC2208 é normalmente *ativo em nível baixo*.

## Comportamento

O firmware:
- liga o driver
- envia `STEP_COUNT` pulsos para frente
- espera `STEP_DWELL_MS`
- envia `STEP_COUNT` pulsos para trás
- repete

### Ajustes rápidos

No `main/main.c` você pode mudar:

- `STEP_PULSE_US` — largura do pulso do STEP
- `STEP_PERIOD_US` — intervalo entre pulsos
- `STEP_COUNT` — quantidade de passos por bloco
- `STEP_DWELL_MS` — pausa entre blocos

## Build no Termux/Ubuntu

```bash
source /root/esp-idf/export.sh
idf.py set-target esp32s3
idf.py build
```

Se o seu `idf.py` estiver em outro lugar, use o caminho direto.

## Flash

Exemplo básico:

```bash
idf.py -p /dev/ttyUSB0 flash monitor
```

Se você estiver usando um adaptador USB/serial pelo Android/OTG, ajuste a porta para o device correto.

## Observação importante

Esse projeto é propositalmente simples: ele não usa UART do TMC2208 nem microstepping dinâmico. É só a base para validar:

- STEP
- DIR
- ENABLE
- sentido de rotação
- pulso mínimo de funcionamento
