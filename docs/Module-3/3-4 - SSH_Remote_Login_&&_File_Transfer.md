# SSH Remote Login && File Transfer

# SSH Remote Login & File Transfer

**Tip:** For the configured system, the default username is typically `jetson`, and the initial password is `yahboom`.

## 1. Remote Login Guide

First, you need to determine the IP address of your device.

### Method 1: Using the System Interface

You can often find the IP address directly on the interface during the initial system setup process.

Alternatively, open a terminal (e.g., by pressing `Ctrl+Alt+T`) and execute the command `ifconfig`.
Locate the IP address associated with your wired network interface, typically `eth0`. If you are using a wireless network adapter, look for the address under `wlan`.

### Method 2: Via Your Wireless Router

Access your wireless router's administration interface to find the IP address assigned to your device.

Once you have the IP address, launch PuTTY. Input your device's IP address and the default SSH port (usually 22). The SSH service is typically enabled by default on the system.

![](/img/docs/3-4/2023040600001.png)

After entering the details, click **Open**. When prompted with a security alert, select **Yes** to proceed.

![](/img/docs/3-4/2023040600002.png)

Provide the username that was configured during the system installation (e.g., `Orin nano`).

![](/img/docs/3-4/2023040600003.png)

Next, input the password for your user account. Please note that the characters you type for the password will not be displayed; simply type it and press `Enter` to access the terminal.

![](/img/docs/3-4/2023040600004.png)

## 2. Remote File Transfer Guide

Transferring files between distinct operating systems, such as Windows and Linux, often requires a specialized approach due to their different file systems. The SSH service facilitates this cross-system file transfer. This guide utilizes WinSCP software, known for its user-friendliness.

In WinSCP, create a new site by entering the IP address of your device, your username, and your password. Then click **Login**. If these connection details are likely to remain unchanged, you can save the site for quicker access in the future.

![](/img/docs/3-4/2023040600006.png)

Upon successful login, you will see an interface with two panes: the left pane displays your local Windows file system, while the right pane shows the remote Linux system's file directories (e.g., your Orin nano's folders). This dual-pane view allows for easy file management, including drag-and-drop transfers, or right-clicking files for options like moving or deleting.

![](/img/docs/3-4/2023040600007.png)

File transfer can be performed using three main methods:

*   **Method 1: Drag and Drop**
    Simply drag a file from the left (Windows) pane to the right (Linux) pane, or vice versa. The system will then automatically initiate the file copy and transfer.

*   **Method 2: F5 Key**
    Select the desired file with your mouse and then press the `F5` key. This action will copy the selected file to the opposite pane.

*   **Method 3: Right-Click Context Menu**
    Right-click a selected file.
    *   To transfer a file from your Windows computer to the remote Orin nano, choose the **Upload** option from the context menu.

![](/img/docs/3-4/2023040600008.png)

A prompt may appear; you can opt to disable future prompts and then click **OK** to proceed with the file transfer.

*   Conversely, to transfer files from the Orin nano to your Windows computer, right-click the file on the remote system and select **Download**.

![](/img/docs/3-4/2023040600010.png)