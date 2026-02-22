---
title: 03-Installing Jetson Environment
sidebar_position: 3
---

# 03-Installing Jetson Environment

This tutorial installs NVIDIA component packages on a board that already boots normally.

:::note
If your shipped SSD already includes the factory image and you do not need a clean reflash, use this tutorial.
:::

## 1. Prerequisites

Prepare:

- Stable internet connection (host and target)
- Jetson board booting to desktop
- DC power adapter
- DisplayPort cable + monitor
- USB Type-C cable to host machine/VM
- Ethernet cable

If you use the provided VM image, use its default credentials as documented by your kit provider.

## 2. Hardware and Connection Mode

Connect the board with DC power, DP, Ethernet, and Type-C.

:::note
Do not enter recovery (flashing) mode for this tutorial.
:::

![Screenshot](/img/docs/hh101/02-Basics/2-3/image-20250109180831845.png)

## 3. VM/Host USB Pass-Through

If using VMware Workstation 17 Pro, attach the Jetson USB device to the VM from the VMware menu.

Example menu path:

- `Virtual Machine -> Removable Devices -> NVIDIA Linux for Tegra`

![Screenshot](/img/docs/hh101/02-Basics/2-3/image-20250123100937979.png)

## 4. SDK Manager Workflow

### 4.1 Select Target

Open `SDK Manager` and select your target board model.

For official kit users, choose `Jetson Orin Nano [*GB developer kit version]`.

### 4.2 Step 1

Confirm the target and host options, then click `CONTINUE`.

![Screenshot](/img/docs/hh101/02-Basics/2-3/image-20250116194316209-1737442070436-3.png)

### 4.3 Step 2

Select the component packages you need. SDK Manager downloads them automatically.

![Screenshot](/img/docs/hh101/02-Basics/2-3/image-20250116194450017.png)

If prompted, enter the VM password.

![Screenshot](/img/docs/hh101/02-Basics/2-3/image-20250116194516622.png)

### 4.4 Step 3

Enter the username and password configured on the Jetson device.

![Screenshot](/img/docs/hh101/02-Basics/2-3/image-20250116194558310.png)

Wait for package installation to complete.

![Screenshot](/img/docs/hh101/02-Basics/2-3/image-20250116194622869.png)

If installation fails, reboot the board and retry SDK Manager.

![Screenshot](/img/docs/hh101/02-Basics/2-3/image-20250116194818632.png)

![Screenshot](/img/docs/hh101/02-Basics/2-3/image-20250116200418604.png)

### 4.5 Step 4

Verify completion in SDK Manager.

![Screenshot](/img/docs/hh101/02-Basics/2-3/image-20250116203345930.png)

## 5. Environment Verification

Install `jtop`:

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install python3-pip -y
sudo pip3 install -U jetson-stats
```

Reboot after installation.

Set max power mode:

```bash
sudo nvpmodel -m 2 # Jetson Orin Nano
sudo nvpmodel -m 0 # Jetson Orin NX
```

Enable max clocks:

```bash
sudo jetson_clocks
```

Check status:

```bash
jtop
```

![Screenshot](/img/docs/hh101/02-Basics/2-3/88d179920f811d989b1c909d6adb59b9.png)

![Screenshot](/img/docs/hh101/02-Basics/2-3/image-20250121150457398.png)

![Screenshot](/img/docs/hh101/02-Basics/2-3/image-20250121150858152.png)
