# Untitled

# VNC Configuration

This guide provides instructions for configuring VNC on your device.

**Tip:** If you are using a pre-configured image, it likely has a default username of `Jetson` and an initial password of `yahboom`. If VNC is already set up, you can skip directly to the "Connect to VNC Server" section and log in using the device's current IP address.

## 1. Install Vino

Begin by updating your system's package list, then install the Vino VNC server application.

```bash
sudo apt update
```

![](/img/docs/3-5/2023040600001.png)

```bash
sudo apt install vino
```

![](/img/docs/3-5/2023040600002.png)

## 2. Enable VNC Service

This section outlines how to enable the VNC service and configure its basic settings.

First, create a symbolic link to activate the Vino server service:

```bash
sudo ln -s ../vino-server.service /usr/lib/systemd/user/graphical-session.target.wants
```

![](/img/docs/3-5/2023040600003.png)

Next, configure the VNC server to disable connection prompts and encryption requirements:

```bash
gsettings set org.gnome.Vino prompt-enabled false
gsettings set org.gnome.Vino require-encryption false
```

![](/img/docs/3-5/2023040600004.png)

To ensure the VNC server can be enabled, you need to modify the `org.gnome.Vino.gschema.xml` file. Open the file using a text editor:

```bash
sudo vi /usr/share/glib-2.0/schemas/org.gnome.Vino.gschema.xml
```

![](/img/docs/3-5/2023040600005.png)

Add the following XML content to the end of the file, then save and exit:

```xml
<key name='enabled' type='b'>
  <summary>Enable remote access to the desktop</summary>
  <description>
    If true, allows remote access to the desktop via the RFB protocol. Users on remote machines may then connect to the desktop using a VNC viewer.
  </description>
  <default>false</default>
</key>
```

![](/img/docs/3-5/2023040600006.png)

After modifying the schema file, compile the Glib schemas to apply the changes:

```bash
sudo glib-compile-schemas /usr/share/glib-2.0/schemas
```

At this point, the screen sharing panel in the unit control center should be functional. However, Vino still needs to be launched. You can manually start the Vino server for the current session using:

```bash
/usr/lib/vino/vino-server
```

![](/img/docs/3-5/2023040600007.png)

Manually starting the server each time can be inconvenient. The next steps will configure VNC for automatic startup.

## 3. Set VNC Login Password

Configure the VNC server to use password authentication and set your desired password. Replace `'thepassword'` with your actual password.

```bash
gsettings set org.gnome.Vino authentication-methods "['vnc']"
gsettings set org.gnome.Vino vnc-password $(echo -n 'thepassword' |base64)
```

![](/img/docs/3-5/2023040600008.png)

## 4. Restart and Configure Automatic VNC Server Startup

Reboot your machine to apply the changes and verify the VNC settings.

```bash
sudo reboot
```

The VNC server typically becomes available only after a local user logs into the Jetson. To make VNC automatically available upon system boot, irrespective of local login, you can enable automatic login in your system settings and configure Vino to start with the user session.

First, enable the Vino VNC server:

```bash
gsettings set org.gnome.Vino enabled true
```

Next, create an autostart directory and a desktop entry file for the Vino server:

```bash
mkdir -p ~/.config/autostart
vi ~/.config/autostart/vino-server.desktop
```

Add the following content to the `vino-server.desktop` file, then save and exit:

```ini
[Desktop Entry]
Type=Application
Name=Vino VNC server
Exec=/usr/lib/vino/vino-server
NoDisplay=true
```

![](/img/docs/3-5/2023040600009.png)

**Note:** If your system requires a user password before entering the desktop, the above autostart script will not execute until the desktop session is initiated. For seamless VNC auto-start, it is recommended to configure the system for automatic user login to the desktop.

## Connect to VNC Server

To connect to your VNC server using a VNC viewer application:

1.  **Find the IP Address:** Determine the IP address of your device. For example, the IP address found here is `192.168.1.195`.
2.  **Enter IP Address:** In your VNC viewer, enter the device's IP address and click "OK" or "Connect."

![](/img/docs/3-5/2023040600010.png)

3.  **Enter Password:** Double-click on the corresponding VNC user (if prompted) and enter the VNC password you set earlier.

![](/img/docs/3-5/2023040600011.png)

4.  **Access VNC Interface:** Once authenticated, you will be connected to the VNC desktop interface.

![](/img/docs/3-5/2023040600012.png)

## VNC Usage Considerations for Jetson Orin Nano

Keep the following points in mind when using VNC with your Jetson Orin Nano:

*   **Wi-Fi Sharing:** Whenever you connect to a new Wi-Fi network, you must enable Wi-Fi sharing from the settings page.

![](/img/docs/3-5/2023040600013.png)

*   **Display Cable Connection:** To access the Jetson Orin Nano desktop via VNC, it is essential to have a DP (DisplayPort) display cable properly connected to the device. Without a connected physical display, VNC desktop access may not function correctly.

![](/img/docs/3-5/2023040600014.png)