---
title: GPIO Library Installation
sidebar_position: 0
---

# GPIO library installation

GPIO library installation 1. Install Jetson.GPIO library 1.1. Automatic installation 1.2. Manual installation 2. Set user permissions 3. Custom rule file 3.1. Copy the rule file 3.2. Reload udev rules 4. Set the motherboard model 5. References

## 1. Install Jetson.GPIO library

The system installs Jetson.GPIO library by default, so you can skip this step.

### 1.1. Automatic installation

```bash
xxxxxxxxxx
sudo pip3 install Jetson.GPIO
```

```bash
xxxxxxxxxx
```

```bash
sudo pip3 install Jetson.GPIO
```

![image-20250110101719371](/img/docs/jetson/04-GPIOcontrolcourse/4-2/image-20250110101719371.png)

### 1.2. Manual installation

It is recommended to use the automatic installation method. Manual installation may not be the latest version.

```bash
xxxxxxxxxx
git clone https://github.com/NVIDIA/jetson-gpio
```

```bash
xxxxxxxxxx
```

```bash
git clone https://github.com/NVIDIA/jetson-gpio
```

```bash
xxxxxxxxxx
cd ~/jetson-gpio/
```

```bash
xxxxxxxxxx
```

```bash
cd ~/jetson-gpio/
```

```bash
xxxxxxxxxx
sudo python3 setup.py install
```

```bash
xxxxxxxxxx
```

```bash
sudo python3 setup.py install
```

![image-20250110102057706](/img/docs/jetson/04-GPIOcontrolcourse/4-2/image-20250110102057706.png)

## 2. Set user permissions

Allow the current system user to access and use the Jetson.GPIO library: where jetson is the system user name

```bash
xxxxxxxxxx
sudo groupadd -f -r gpio
sudo usermod -a -G gpio jetson
```

```bash
xxxxxxxxxx
```

```bash
sudo groupadd -f -r gpio
```

```bash
sudo usermod -a -G gpio jetson
```

![image-20250110102615802](/img/docs/jetson/04-GPIOcontrolcourse/4-2/image-20250110102615802.png)

## 3. Custom rule file

### 3.1. Copy the rule file

```bash
xxxxxxxxxx
cd ~/jetson-gpio/
sudo cp lib/python/Jetson/GPIO/99-gpio.rules /etc/udev/rules.d/
```

```bash
xxxxxxxxxx
```

```bash
cd ~/jetson-gpio/
```

```bash
sudo cp lib/python/Jetson/GPIO/99-gpio.rules /etc/udev/rules.d/
```

### 3.2. Reload udev rules

```bash
xxxxxxxxxx
sudo udevadm control --reload-rules && sudo udevadm trigger
```

```bash
xxxxxxxxxx
```

```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
```

![image-20250110102901779](/img/docs/jetson/04-GPIOcontrolcourse/4-2/image-20250110102901779.png)

## 4. Set the motherboard model

Currently, Jetpack6.1 does not set the motherboard model in advance. You need to set the motherboard model in the terminal before controlling GPIO each time:

```bash
xxxxxxxxxx
export JETSON_MODEL_NAME=JETSON_ORIN_NANO
```

```bash
xxxxxxxxxx
```

```bash
export JETSON_MODEL_NAME=JETSON_ORIN_NANO
```

## 5. References

https://github.com/NVIDIA/jetson-gpio
