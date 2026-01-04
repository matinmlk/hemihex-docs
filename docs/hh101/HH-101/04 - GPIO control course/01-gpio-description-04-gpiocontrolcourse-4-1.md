---
title: GPIO Description
sidebar_position: 0
---

# GPIO Description

GPIO Description 1. GPIO numbering 2. GPIO pins 2.1. GPIO.BOARD mode 2.2. GPIO.BCM mode 3. Other pins

The Jetson.GPIO library of the Jetson series motherboards allows developers to interact with external hardware devices through the 40-pin GPIO pins.

## 1. GPIO numbering

Jetson.GPIO library supports two common numbering modes: BOARD and BCM mode

:::note
Note: The tutorial only demonstrates GPIO.BOARD mode
:::

| Mode | Numbering method |
| --- | --- |
| GPIO.BOARD | Numbering based on physical pins (1-40) |
| GPIO.BCM | GPIO logical numbering based on Broadcom chips (directly corresponding to the GPIO channels on the hardware chip) |

## 2. GPIO pins

### 2.1. GPIO.BOARD mode

One-to-one correspondence between BOARD mode pins and 40Pin pins on the motherboard:

![image-20250108121230644](/img/docs/jetson/04-GPIOcontrolcourse/4-1/image-20250108121230644.png)

### 2.2. GPIO.BCM mode

One-to-one correspondence between BCM mode pins and 40Pin pins on the motherboard:

![image-20250109153031466](/img/docs/jetson/04-GPIOcontrolcourse/4-1/image-20250109153031466.png)

## 3. Other pins

12Pin pin description under the Jetson core board:

![image-20250109161945494](/img/docs/jetson/04-GPIOcontrolcourse/4-1/image-20250109161945494.png)
