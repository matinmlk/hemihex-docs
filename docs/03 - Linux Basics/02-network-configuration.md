---
title: Network Configuration
sidebar_position: 0
---

# Network configuration

Network configuration 1. WIFI mode 1.1. Connect to WiFi 1.3. Set static IP 2. Hotspot mode 2.1. Create a hotspot 2.2, Hotspot information

:::note
WiFi and hotspot modes require the use of a wireless network card. Before making the following settings, check whether the wireless network card and antenna are installed!
:::

```bash
It is recommended to switch networks by connecting to the display screen. Once the network is switched to a new network, the system needs to re-enable network sharing for the new network before VNC remote
```

```bash
It is recommended to switch networks by connecting to the display screen. Once the network is switched to a new network, the system needs to re-enable network sharing for the new network before VNC remote
```

## 1. WIFI mode

### 1.1. Connect to WiFi

Select the menu option in the upper right corner of the system desktop → WiFi options → Wi-Fi Settings:

![image-20250110142107961](/img/docs/jetson/03-LinuxBasics/3-2/image-20250110142107961.png)

Select the WiFi you want to connect to: If the WiFi signal is very weak, check whether the antenna is not installed or the signal in the environment is poor

![image-20250110142144574](/img/docs/jetson/03-LinuxBasics/3-2/image-20250110142144574.png)

After entering the password, click Connect : ### 1.2. Check WiFi information

![image-20250110142252074](/img/docs/jetson/03-LinuxBasics/3-2/image-20250110142252074.png)

![image-20250110142302019](/img/docs/jetson/03-LinuxBasics/3-2/image-20250110142302019.png)

![image-20250110142334374](/img/docs/jetson/03-LinuxBasics/3-2/image-20250110142334374.png)

Click the settings icon of the connected WiFi:

![image-20250110142811857](/img/docs/jetson/03-LinuxBasics/3-2/image-20250110142811857.png)

The terminal can use the following command to view the IP addresses of all networks: enP8p1s0 is the IP connected by the network cable, and wlP1p1s0 is the IP connected by WiFi

```bash
ifconfig
```


```bash
ifconfig
```

![image-20250110143429233](/img/docs/jetson/03-LinuxBasics/3-2/image-20250110143429233.png)

### 1.3. Set static IP

Click the setting icon of the connected WiFi to modify the IPv4 option:

Address: Fill in the required fixed IP address, which needs to be in the assignable IP address range

Netmask: Fill in 255.255.255.0

Gateway: Fill in the WiFi default gateway address

![image-20250110151740342](/img/docs/jetson/03-LinuxBasics/3-2/image-20250110151740342.png)

After completion, reconnect WiFi to take effect:

![image-20250110151939300](/img/docs/jetson/03-LinuxBasics/3-2/image-20250110151939300.png)

## 2. Hotspot mode

The wireless network card needs to support hotspot to enable hotspot mode.

```bash
Configure the hotspot mode on the desktop system. The hotspot will be automatically turned off after the system restarts. Users who need it can find the tutorial on how to start the hotspot on Ubuntu 22.04
```


```bash
Configure the hotspot mode on the desktop system. The hotspot will be automatically turned off after the system restarts. Users who need it can find the tutorial on how to start the hotspot on Ubuntu 22.04
```

### 2.1. Create a hotspot

Enter WiFi settings and select Turn On Wi-Fi Hotspot...

![image-20250110143648422](/img/docs/jetson/03-LinuxBasics/3-2/image-20250110143648422.png)

### 2.2, Hotspot information

Hotspot name: Jetson_Orin_Hot (customizable)

Hotspot password: 12345678 (customizable)

Hotspot mode default IP: 10.42.0.1

![image-20250110143824381](/img/docs/jetson/03-LinuxBasics/3-2/image-20250110143824381.png)

![image-20250110145951438](/img/docs/jetson/03-LinuxBasics/3-2/image-20250110145951438.png)
