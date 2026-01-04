---
title: VNC Remote Control
sidebar_position: 0
---

# VNC remote control

Tutorial to configure the built-in screen sharing of Ubuntu22.04 system for VNC remote control.

:::note
Windows computer needs to download and install VNC Viewer in advance and the remote device and the remote device are in the same LAN
:::

## 1. VNC Viewer

### 1.1. VNC download

Official website download address: https://www.realvnc.com/en/connect/download/viewer/

![Screenshot](/img/docs/jetson/03-LinuxBasics/3-4/1731245408722-e081c41b-fcc9-4d51-a8b7-5786d9834797-1735196229353-7.png)

### 1.2. VNC Installation

Run VNC-Viewer-xxx.exe as an administrator:

![image.png](/img/docs/jetson/03-LinuxBasics/3-4/1.png)

![image.png](/img/docs/jetson/03-LinuxBasics/3-4/1731245722779-c232e01d-d48f-4bac-bc6c-90bcfc2b101b.webp)

![image.png](/img/docs/jetson/03-LinuxBasics/3-4/1731245755907-64df35f9-1065-4d8c-931a-f358c687dc8f.webp)

![image.png](/img/docs/jetson/03-LinuxBasics/3-4/1731245794795-9efa7e97-85ea-4c79-b598-f17a6c46ad8b.webp)

 ### 1.3. Use VNC

![image.png](/img/docs/jetson/03-LinuxBasics/3-4/1731245821487-a4d059af-3153-444c-a66c-7dc488f77e74.webp)

![image.png](/img/docs/jetson/03-LinuxBasics/3-4/1731245859996-34958ed2-fbbf-4e7c-b925-1cd25adc6061.webp)

![image.png](/img/docs/jetson/03-LinuxBasics/3-4/1731245899250-ae4f4eec-d90b-4a53-8003-94a56814f982.webp)

![image.png](/img/docs/jetson/03-LinuxBasics/3-4/1731246035780-cef8ec3f-f829-41bd-b385-c34ddb0a4df8.webp)

![image.png](/img/docs/jetson/03-LinuxBasics/3-4/1731246075083-73406283-f3f0-42fb-9101-86797fcfe99f.webp)

## 2. System Settings (Jetson)

### 2.1. Enable desktop remote

#### 2.1.1. Sharing

Settings → Sharing

![Screenshot](/img/docs/jetson/03-LinuxBasics/3-4/image-20241226150132128.png)

#### 2.1.2. Remote Desktop

Turn on the remote desktop and enable the traditional VNC protocol (need to check the password required): the access password can be modified by yourself!

![Screenshot](/img/docs/jetson/03-LinuxBasics/3-4/image-20241226150351236.png)

![Screenshot](/img/docs/jetson/03-LinuxBasics/3-4/image-20241226150404221.png)

#### 2.1.3, Media Sharing

You need to check this option every time you switch networks and turn on the switch of the new network:

![Screenshot](/img/docs/jetson/03-LinuxBasics/3-4/image-20241226150556441.png)

#### 2.1.4 Remote Login

Turn on remote login:

![Screenshot](/img/docs/jetson/03-LinuxBasics/3-4/image-20241226150706947.png)

### 2.2, Fixed remote password

You can perform VNC remote control by completing the above settings, but the access password of the Jetson motherboard will change every time it restarts. The fixed password needs to be operated as follows!

#### Passwords and Keys

Enter Passwords and Keys to set no key:

![Screenshot](/img/docs/jetson/03-LinuxBasics/3-4/image-20241226150816361.png)

Select the default key to modify the password:

![Screenshot](/img/docs/jetson/03-LinuxBasics/3-4/image-20241226150953862.png)

Enter the current password:

![Screenshot](/img/docs/jetson/03-LinuxBasics/3-4/image-20241226151041911.png)

Set an empty key: Submit without filling in any content

![Screenshot](/img/docs/jetson/03-LinuxBasics/3-4/image-20241226151117841.png)

![Screenshot](/img/docs/jetson/03-LinuxBasics/3-4/image-20241226151130227.png)

### 2.3, Start VNC automatically after booting

After completing the above operations, the Jetson motherboard cannot be remotely accessed by VNC after the screen is locked. We can follow the following operations to solve the remote problem of locked screen.

#### Desktop extension manager

Install desktop extension manager:

```bash
sudo apt install gnome-shell-extension-manager -y
```

Get the gnome-shell version number:

```bash
gnome-shell --version
```


![Screenshot](/img/docs/jetson/03-LinuxBasics/3-4/image-20241226152159374.png)

Download the plug-in that allows remote access under lock screen according to the version number:

```bash
Official website: https://extensions.gnome.org/extension/4338/allow-locked-remote-desktop/
```


```bash
Official website: https://extensions.gnome.org/extension/4338/allow-locked-remote-desktop/
```

![Screenshot](/img/docs/jetson/03-LinuxBasics/3-4/image-20241226152116720.png)

Install/enable plug-in: Users need to enter the file location to install

```bash
gnome-extensions install allowlockedremotedesktopkamens.us.v9.shell-extension.zip
```


```bash
sudo gnome-extensions enable allowlockedremotedesktop@kamens.us
```

![Screenshot](/img/docs/jetson/03-LinuxBasics/3-4/image-20250110100353197.png)

Restart the system: open Extension Manager to enable the corresponding function (find it in the Ubuntu system application)

![Screenshot](/img/docs/jetson/03-LinuxBasics/3-4/image-20250110100556772.png)

![Screenshot](/img/docs/jetson/03-LinuxBasics/3-4/image-20250110100610568.png)

## 3. VNC remote control

VNC Viewer input motherboard IP:

![Screenshot](/img/docs/jetson/03-LinuxBasics/3-4/image-20241226153719054.png)

Fill in the motherboard system password:

![Screenshot](/img/docs/jetson/03-LinuxBasics/3-4/image-20241226153812612.png)

![Screenshot](/img/docs/jetson/03-LinuxBasics/3-4/image-20241226154140728.png)

## Frequently Asked Questions

### VNC Remote Display Reconnection

#### Reconnection Phenomenon

![Screenshot](/img/docs/jetson/03-LinuxBasics/3-4/image-20241226153930074.png)

#### Solution

Modify the options of the corresponding remote device → Specify remote image quality

![Screenshot](/img/docs/jetson/03-LinuxBasics/3-4/image-20241226154023304.png)

![Screenshot](/img/docs/jetson/03-LinuxBasics/3-4/image-20241226154110695.png)

![Screenshot](/img/docs/jetson/03-LinuxBasics/3-4/image-20241226154145590.png)

### VNC remote switch uppercase and lowercase

Enter Settings → Compose Key → Caps Lock: Set to Caps Lock to switch uppercase and lowercase input

![Screenshot](/img/docs/jetson/03-LinuxBasics/3-4/image-20241230103417649.png)
