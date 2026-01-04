---
title: I2C Communication
sidebar_position: 0
---

# I2C communication

I2C communication 1. GPIO pin diagram 2. I2C test 2.1. Installation dependencies 2.2. I2C device 2.2.1. Query I2C bus 2.2.2. Query I2C device 2.3. Run the program 3. Experimental results

## 1. GPIO pin diagram

Use 0.91-inch OLED to test the I2C communication function and connect according to the following wiring:

:::warning
Do not connect incorrectly or cause pin short circuits. Mistakes may cause damage to the motherboard hardware!
:::

![image-20250110112330754](/img/docs/jetson/04-GPIOcontrolcourse/4-4/image-20250110112330754.png)

## 2. I2C test

### 2.1. Installation dependencies

```bash
sudo pip3 install smbus
sudo pip3 install Adafruit_SSD1306
```

```bash
sudo pip3 install smbus
```

```bash
sudo pip3 install Adafruit_SSD1306
```

![image-20250110113436522](/img/docs/jetson/04-GPIOcontrolcourse/4-4/image-20250110113436522.png)

### 2.2. I2C device

During normal development, we need to find the device bus and device address where the I2C device is mounted.

#### 2.2.1. Query I2C bus

Enter the following command in the terminal to list all busses of the device:

```bash
i2cdetect -l
```

```bash
xxxxxxxxxx
```

```bash
i2cdetect -l
```

#### 2.2.2. Query I2C device

Enter the following command in the terminal to list I2C devices under the specified bus: The I2C address corresponding to oled is 0x3c

```bash
i2cdetect -y -r *
```

```bash
xxxxxxxxxx
```

```bash
i2cdetect -y -r *
```

![image-20250110113848297](/img/docs/jetson/04-GPIOcontrolcourse/4-4/image-20250110113848297.png)

### 2.3. Run the program

oled_i2c.py is not included in the jetson-gpio library:

```bash
cd ~/jetson-gpio/samples/
```

```bash
xxxxxxxxxx
```

```bash
cd ~/jetson-gpio/samples/
```

```bash
python3 oled_i2c.py
```

```bash
xxxxxxxxxx
```

```bash
python3 oled_i2c.py
```

## 3. Experimental results

After starting the program, OLED will display system information such as system CPU usage, system time, and memory usage:

![image-20250110114104871](/img/docs/jetson/04-GPIOcontrolcourse/4-4/image-20250110114104871.png)

![image-20250110114410547](/img/docs/jetson/04-GPIOcontrolcourse/4-4/image-20250110114410547.png)
