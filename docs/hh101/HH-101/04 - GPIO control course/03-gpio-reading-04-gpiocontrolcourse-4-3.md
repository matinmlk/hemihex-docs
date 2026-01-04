---
title: GPIO Reading
sidebar_position: 0
---

# GPIO reading

GPIO reading 1. GPIO pin diagram 2. Run the program 3. Program effect

## 1. GPIO pin diagram

GPIO.BOARD 12 pin corresponds to GPIO.BCM 18 pin:

![image-20250110104406267](/img/docs/jetson/04-GPIOcontrolcourse/4-3/image-20250110104406267.png)

## 2. Run the program

```bash
cd ~/jetson-gpio/samples
```

```bash
cd ~/jetson-gpio/samples
```

```bash
export JETSON_MODEL_NAME=JETSON_ORIN_NANO
```

```bash
xxxxxxxxxx
```

```bash
export JETSON_MODEL_NAME=JETSON_ORIN_NANO
```

```bash
python3 simple_input.py
```

```bash
xxxxxxxxxx
```

```bash
python3 simple_input.py
```

## 3. Program effect

Use Dupont wire to connect GPIO.BOARD 12 pin to GND and 3.3V pin on the motherboard, and test the reading of high and low levels:

:::warning
Do not connect incorrectly or cause pin short circuit, as mistakes may damage the motherboard hardware!
:::

![image-20250110103545095](/img/docs/jetson/04-GPIOcontrolcourse/4-3/image-20250110103545095.png)
