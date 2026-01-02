---
title: Write SUPER Boot (Official Kit)
sidebar_position: 0
---

# Write SUPER boot

Write SUPER boot 1. Flashing mode 1.1. Hardware connection 1.2. Software connection 2. Write boot 3. Start the system

The purpose of this tutorial is to burn SUPER boot to the Jetson Orin series motherboard (used with Jetpack 6.2 system). There is no need to install a solid-state drive during the burning process. After the burning is completed, install the solid-state drive to the motherboard and start the system to use the factory system that we have set up in advance.

## 1. Flashing mode

### 1.1. Hardware connection

:::note
The DP data cable and network cable can be used without burning the boot, but they will be needed when using the motherboard later
:::

This illustration is based on the official version of Jetson Orin Nano. Users of other versions can refer to it for use (the hardware interface and functional layout are the same).

![Screenshot](/img/docs/jetson/02-Basics/2-8/image-20250121195352195.png)

### 1.2. Software connection

Users need to use the virtual machine we provide to burn SUPER boot. We need to connect the motherboard to the virtual machine so that it can be recognized by the Ubuntu system:

```
Virtual machine username: HemiHex
Virtual machine password: HemiHex
```

```
Virtual machine username: HemiHex
```

```
Virtual machine password: HemiHex
```

![Screenshot](/img/docs/jetson/02-Basics/2-8/image-20250123100406992.png)

![Screenshot](/img/docs/jetson/02-Basics/2-8/image-20250122104757404.png)

## 2. Write boot

Open the terminal, enter the specified folder and run the script: If the burning fails, you can disconnect the motherboard power and reconnect the virtual machine to run the command

```bash
cd ~/jetpack_6.2/Linux_for_Tegra && sudo ./yahboom_flash.sh.x
```

```
xxxxxxxxxx
```

```bash
cd ~/jetpack_6.2/Linux_for_Tegra && sudo ./yahboom_flash.sh.x
```

![Screenshot](/img/docs/jetson/02-Basics/2-8/image-20250122104832985.png)

![Screenshot](/img/docs/jetson/02-Basics/2-8/image-20250122105606210.png)

## 3. Start the system

After the burning boot is successful, install the solid state drive boot system provided by our factory.

![Screenshot](/img/docs/jetson/02-Basics/2-8/image-20250122110640862.png)
