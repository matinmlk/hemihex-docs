# Upgrade to SUPER version

# Upgrading to the SUPER System Version

This guide outlines the process for upgrading to the SUPER system version. The steps cover file download, entering flashing mode, burning the system to the solid-state drive, and finally, booting the system.

This upgrade procedure is compatible with both the Jetson Orin official kit and the Jetson Orin SUB kit. It's important to note that after the SUPER upgrade, only a stripped-down system will remain, meaning other motherboard-specific functionalities or cases might not operate.

**Important Considerations:**
*   The startup process for Jetson series motherboards is highly dependent on the Jetpack version. Using incompatible Jetpack versions can lead to boot failures.
*   For demonstration purposes, this tutorial utilizes a VMware-hosted Ubuntu 22.04 virtual machine.

## 1. File Download

Acquire the necessary files from the official NVIDIA developer portal:
`https://developer.nvidia.com/embedded/jetson-linux-r3643`

**Note:** NVIDIA Jetson Linux 36.4.3 is associated with Jetpack 6.2.

Download the compressed archives for both the **Driver Package (BSP)** and the **Sample Root Filesystem**.

![](/img/docs/2-7/image-20250120150532008.png)

## 2. Flashing Mode

### 2.1 Hardware Connection

To prepare for flashing, perform the following hardware connections:

1.  Bridge the FC REC and GND pins located beneath the core board using a jumper cap. (The core board can remain uninstalled; the image is for better visibility.)
2.  Connect the Jetson Orin motherboard to a DC power adapter, a DisplayPort (DP) cable, an Ethernet cable, and a USB Type-C data cable. The Type-C cable should connect to your computer.

![](/img/docs/2-7/image-20250120171038907.png)

### 2.2 Software Connection

Verify that the motherboard is successfully connected to your Ubuntu system. Execute the `lsusb` command in your terminal. A successful connection will display information indicating `NVIDIA Corp. APX`.

```bash
lsusb
```

![](/img/docs/2-7/image-20250120154019235.png)

## 3. Burn the System

### 3.1 Unzip Files

Navigate to your download directory and open a terminal. Then, decompress the downloaded archives and change into the designated folder using the following commands:

```bash
tar xf Jetson_Linux_R36.4.3_aarch64.tbz2
sudo tar xpf Tegra_Linux_Sample-Root-Filesystem_R36.4.3_aarch64.tbz2 -C Linux_for_Tegra/rootfs/
cd Linux_for_Tegra/
```

![](/img/docs/2-7/image-20250120151802071.png)

### 3.2 Run Scripts

Execute the following prerequisite script:

```bash
sudo ./tools/l4t_flash_prerequisites.sh
```

![](/img/docs/2-7/image-20250120151922577.png)

Next, apply the binaries:

```bash
sudo ./apply_binaries.sh
```

![](/img/docs/2-7/image-20250120152138271.png)

### 3.3 Flash System to Solid-State Drive (SSD)

Use the following command to burn the system image to the NVMe SSD:

```bash
sudo ./tools/kernel_flash/l4t_initrd_flash.sh --external-device nvme0n1p1 -c tools/kernel_flash/flash_l4t_t234_nvme.xml -p "-c bootloader/generic/cfg/flash_t234_qspi.xml" --showlogs --network usb0 jetson-orin-nano-devkit-super internal
```

**Note:** This command is applicable for flashing the system to the SSD for both Jetson Orin Nano and Jetson Orin NX modules.

![](/img/docs/2-7/image-20250120152242874.png)

**Important:** During the flashing process, users must ensure the device remains connected to the virtual machine. Failure to do so may result in a connection timeout.

![](/img/docs/2-7/image-20250120155459747.png)

![](/img/docs/2-7/image-20250120155405246.png)

## 4. Start the System

Once the system has been successfully flashed, follow these steps to boot your device:

1.  Disconnect the motherboard's power supply (unplug both the DC power adapter and the Type-C data cable).
2.  Remove the jumper cap that was used to short the FC REC and GND pins beneath the core board.
3.  After confirming that these steps are complete, reconnect the DC power adapter and the DisplayPort (DP) data cable (to your monitor) to power on and start the system.

![](/img/docs/2-7/image-20250120161524122.png)

![](/img/docs/2-7/image-20250120162024278-1737361225177-3.png)

![](/img/docs/2-7/image-20250120162037749.png)