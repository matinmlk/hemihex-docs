---
title: Remote File Transfer
sidebar_position: 0
---

# Remote file transfer

Remote file transfer 1. MobaXterm 2. MobaXterm installation 2.1. Download MobaXterm 2.2. Install MobaXterm 3. Use MobaXterm 4. MobaXterm: SFTP remote

## 1. MobaXterm

MobaXterm is a powerful remote tool that integrates SHH, VNC, FTP and other remote tools.

## 2. MobaXterm installation

Official website: https://mobaxterm.mobatek.net/

![Screenshot](/img/docs/jetson/03-LinuxBasics/3-5/image-20250111153045145.png)

### 2.1. Download MobaXterm

Select the free version to download:

![Screenshot](/img/docs/jetson/03-LinuxBasics/3-5/image-20250111153133358.png)

Select the installation version to download:

![Screenshot](/img/docs/jetson/03-LinuxBasics/3-5/image-20250111153236445.png)

### 2.2. Install MobaXterm

Unzip the compressed package downloaded from the official website, open the MobaXterm_installer_24.4.msi file to install:

![Screenshot](/img/docs/jetson/03-LinuxBasics/3-5/image-20250111153544943.png)

Agree to the agreement:

![Screenshot](/img/docs/jetson/03-LinuxBasics/3-5/image-20250111153613384.png)

Select the software installation location: the default location is recommended

![Screenshot](/img/docs/jetson/03-LinuxBasics/3-5/image-20250111153637165.png)

Official installation:

![Screenshot](/img/docs/jetson/03-LinuxBasics/3-5/image-20250111153705279.png)

![Screenshot](/img/docs/jetson/03-LinuxBasics/3-5/image-20250111153718414.png)

Complete installation:

![Screenshot](/img/docs/jetson/03-LinuxBasics/3-5/image-20250111153808673.png)

## 3. Use MobaXterm

Find the MobaXterm icon on the desktop and open it:

![Screenshot](/img/docs/jetson/03-LinuxBasics/3-5/image-20250111160841077.png)

![Screenshot](/img/docs/jetson/03-LinuxBasics/3-5/image-20250111160906208.png)

## 4. MobaXterm: SFTP remote

Select Session → FTP : Fill in the remote device IP and username

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
: If MobaXterm uses SSH remotely, it will automatically use SFTP remote login in the sidebar
:::

![Screenshot](/img/docs/jetson/03-LinuxBasics/3-5/image-20250111170521375.png)

![Screenshot](/img/docs/jetson/03-LinuxBasics/3-5/image-20250111170548130.png)

![Screenshot](/img/docs/jetson/03-LinuxBasics/3-5/image-20250111171016442.png)
