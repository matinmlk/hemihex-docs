# Network configuration

# Network Configuration

This guide covers essential network configuration steps for your system, including remote access, system updates, and connecting to both wireless and wired networks.

## 1. Remote Login

To access your system remotely, you can use various tools. Popular options include PuTTY, SSH, and Xshell. This section uses PuTTY as an example.

**Troubleshooting Remote Access:**
If you encounter issues with remote access, verify connectivity by attempting to ping the target device from your computer.

*   To find the IP address on a Linux system (like the Jetson Orin Nano), use the command: `ifconfig`
*   To find the IP address on a Windows computer, use the command: `ipconfig`

Once you know the remote device's IP address, you can test connectivity with a ping command (e.g., `ping 192.168.1.XX`, replacing `192.168.1.XX` with the actual IP address).

**Preventing PuTTY Session Drops:**
If your PuTTY sessions frequently disconnect, you can configure it to send keep-alive packets:

![](/img/docs/3-3/2023040600001.png)

![](/img/docs/3-3/2023040600002.png)

1.  Open PuTTY and navigate to the **Connection** section in the left-hand menu.
2.  On the right side, locate the setting to "Send null packets to keep session alive" and set the value to `10`. This configures PuTTY to send a small, empty packet every ten seconds to maintain an active connection.

## 2. System Source Updates

It is generally recommended to update system sources after initial installation. However, the Jetson Orin Nano utilizes an `aarch64` architecture with Ubuntu 20.04.2 LTS, which differs from standard AMD architecture Ubuntu systems. Finding reliable domestic sources for this specific configuration can be challenging, so it is generally not advised to switch from the default sources.

This guide proceeds with updates using the Jetson Orin Nano's default sources. The update process can be lengthy, allowing you to perform other tasks while it runs. It is highly recommended to complete these two actions before starting any AI projects to prevent missing library installations and frequent errors later on.

Execute the following commands:

`sudo apt-get update`

![](/img/docs/3-3/2023040600003.png)

![](/img/docs/3-3/2023040600004.png)

`sudo apt-get full-upgrade`

![](/img/docs/3-3/2023040600005.png)

During the upgrade process, you will be prompted to confirm; enter `Y` to proceed. The `full-upgrade` step can take approximately two hours, depending on your network speed. Please be patient. Upon successful completion, your terminal output should resemble the image below.

![](/img/docs/3-3/2023040600006.png)

At this point, the primary network configuration for system updates is complete.

## 3. Connecting Jetson Orin Nano to Wi-Fi

To connect your Jetson Orin Nano to a wireless network:

![](/img/docs/3-3/2023041500007.png)

1.  Click on the network symbol, typically located in the top bar of the desktop environment.
2.  From the list of available networks, select the desired Wi-Fi network and enter its password. (Example shows connection to "yahboom2" network).

**Obtaining the IP Address (after network connection):**
Once connected to a network, you can find the Jetson Orin Nano's IP address using the following command in the terminal:

`ifconfig`

![](/img/docs/3-3/2023041500008.png)

If connected via Wi-Fi, look for the `wlan0` entry in the output. The `inet` address listed there is your device's Wi-Fi IP address (e.g., `192.168.2.52`).

## 4. Connecting Jetson Orin Nano via Network Cable

If you need to determine the IP address of your Jetson Orin Nano when a display screen is unavailable, you can connect it directly via an Ethernet cable.

**Using an IP Scanning Tool:**
Connect the Jetson Orin Nano, your computer, and a router to the same network. Then, download and use an IP scanning software, such as Advanced IP Scanner, to scan the network for connected devices and their IP addresses.

![](/img/docs/3-3/2023041500010.png)