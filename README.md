# STM32F411 Blackpill

Flash bin to stm32 via stlink

```sh
st-flash write build/BlackPill-CAN.bin 0x08000000
```

## Hardware Setup

Connect all 3v3, gnd

pin|bme280 0|bme280 1|bme280 2|stm32f411
---|---|---|---|---
gnd|gnd|gnd|gnd|gnd
3v3|3v3|3v3|3v3|3v3
