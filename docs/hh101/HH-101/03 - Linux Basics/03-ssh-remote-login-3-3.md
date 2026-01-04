---
title: SSH Remote Login
sidebar_position: 0
---

# SSH remote login

## 1. MobaXterm

MobaXterm is a powerful remote tool that integrates SHH, VNC, FTP and other remote tools.

## 2. MobaXterm installation

Official website: https://mobaxterm.mobatek.net/

![image-20250111153045145](/img/docs/jetson/03-LinuxBasics/3-3/image-20250111153045145.png)

### 2.1. Download MobaXterm

Select the free version to download:

![image-20250111153133358](/img/docs/jetson/03-LinuxBasics/3-3/image-20250111153133358.png)

Select the installation version to download:

![image-20250111153236445](/img/docs/jetson/03-LinuxBasics/3-3/image-20250111153236445.png)

### 2.2. Install MobaXterm

Unzip the compressed package downloaded from the official website, open the MobaXterm_installer_24.4.msi file to install:

![image-20250111153544943](/img/docs/jetson/03-LinuxBasics/3-3/image-20250111153544943.png)

Agree to the agreement:

![image-20250111153613384](/img/docs/jetson/03-LinuxBasics/3-3/image-20250111153613384.png)

Select the software installation location: the default location is recommended

![image-2025011115 3637165](/img/docs/jetson/03-LinuxBasics/3-3/image-20250111153637165.png)

Official installation:

![image-20250111153705279](/img/docs/jetson/03-LinuxBasics/3-3/image-20250111153705279.png)

![image-20250111153718414](/img/docs/jetson/03-LinuxBasics/3-3/image-20250111153718414.png)

Complete installation:

![image-20250111154119018](/img/docs/jetson/03-LinuxBasics/3-3/image-20250111154119018.png)

## 3. Use MobaXterm

Find the MobaXterm icon on the desktop and open it:

![image-20250111160841077](/img/docs/jetson/03-LinuxBasics/3-3/image-20250111160841077.png)

![image-20250111160929218](/img/docs/jetson/03-LinuxBasics/3-3/image-20250111160929218.png)

## 4. MobaXterm: SSH remote

Select Session → SSH : Fill in the remote device IP and username

```text
Default information of Jetson motherboard:
Username: jetson
Password: HemiHex
```

```text
Default information of Jetson motherboard:
```

```text
Username: jetson
```

```text
Password: HemiHex
```

:::note
: When MobaXterm uses SSH remote, it will automatically use SFTP remote login in the sidebar
:::

![image-20250111162217358](/img/docs/jetson/03-LinuxBasics/3-3/image-20250111162217358.png)

![image-20250111162224138](/img/docs/jetson/03-LinuxBasics/3-3/image-20250111162224138.png)

Enter the user password: The password session window will not be displayed. Press Enter after entering the password!

![image-20250111162305871](/img/docs/jetson/03-LinuxBasics/3-3/image-20250111162305871.png)

Save password: It is recommended not to save

![image-20250111162316429](/img/docs/jetson/03-LinuxBasics/3-3/image-20250111162316429.png)

Complete SSH remote:

![image-20250111162409992](/img/docs/jetson/03-LinuxBasics/3-3/image-20250111162409992.png)
