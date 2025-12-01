# SDK Manager – System Flashing Guide  
*For NVIDIA Jetson Devices*

## 1. Download NVIDIA JetPack and SDK Manager

Open NVIDIA’s JetPack page:

https://developer.nvidia.com/embedded/jetpack

Download **SDK Manager** for Ubuntu (18.04 or 20.04).  
You must log in with your NVIDIA Developer account.

![](/img/docs/2023050900001.png)

## 2. Install SDK Manager

Open a terminal and go to the folder where the `.deb` installer was downloaded:

```bash
cd Downloads/
```

![](/img/docs/2023050900002.png)

Install SDK Manager:

```bash
sudo dpkg -i sdkmanager_1.9.2-10889_amd64.deb
```

![](/img/docs/2023050900003.png)

If dependencies are missing, fix them:

```bash
sudo apt --fix-broken install
```

![](/img/docs/2023050900004.png)

## 3. Launch SDK Manager

Open the Ubuntu applications menu and search for **SDK Manager**.

![](/img/docs/2023050900005.png)

Log in using your NVIDIA account.

![](/img/docs/2023050900006.png)

## 4. Put Your Jetson Orin Nano into Recovery Mode

Bridge the **FC REC** and **GND** pins using a jumper.

![](/img/docs/2023050900007.png)
![](/img/docs/2023050900008.png)

Connect HDMI, keyboard, mouse, Micro‑USB, and power.  
The device will automatically enter REC mode.

![](/img/docs/2023050900009.png)

## 5. Select Hardware and JetPack Version

Choose:

- **Target Hardware** → Jetson Orin Nano  
- **JetPack Version** → e.g., 5.1.1


If not detected, ensure REC mode, USB passthrough, and click **Refresh**.

![](/img/docs/2023050900011.png)

Select the correct module (8GB or 16GB):

![](/img/docs/2023050900012.png)

## 6. Select Components

Keep defaults:  
- Jetson OS  
- Jetson SDK Components



Accept the license and continue.

![](/img/docs/2023050900014.png)
![](/img/docs/2023050900015.png)

![](/img/docs/2023050900016.png)

## 7. Initial System Flashing

After OS flashing, your Jetson will reboot.  
Complete initial setup:

- Username  
- Password  
- Network connection  

![](/img/docs/2023050900017.png)

## 8. Install SDK Components

Reconnect the USB so the VM detects the Jetson again.  
Enter the username/password.

Click **Install**.

![](/img/docs/2023050900018.png)

When complete:

![](/img/docs/2023050900020.png)

## 9. Final Step

Remove the jumper from **FC REC → GND**.
