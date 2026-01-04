---
title: VSCode Usage
sidebar_position: 0
---

# VSCode Usage

The tutorial demonstrates the steps to install VSCode and SSH remote on Windows platform.

```text
Users can use this method to remotely control motherboards such as Raspberry Pi and Jetson
```

## 1. Download VSCode

Official website: https://code.visualstudio.com/

![image-20241229141638179](/img/docs/jetson/03-LinuxBasics/3-8/image-20241229141638179.png)

![image-20241229141703922](/img/docs/jetson/03-LinuxBasics/3-8/image-20241229141703922.png)

## 2. Install VSCode

### 2.1. Open the installation package

Open VSCodeUserSetup-x64-xxx.exe as an administrator

![image-20241229141818112](/img/docs/jetson/03-LinuxBasics/3-8/1.png)

### 2.2. Agree to the agreement

![image-20241229142216475](/img/docs/jetson/03-LinuxBasics/3-8/image-20241229142216475.png)

### 2.3. Installation location

It is recommended to select the default installation location of the software:

![image-20241229142324425](/img/docs/jetson/03-LinuxBasics/3-8/image-20241229142324425.png)

### 2.4. Installation options

![image-20241229142425202](/img/docs/jetson/03-LinuxBasics/3-8/image-20241229142425202.png)

![image-20241229142459323](/img/docs/jetson/03-LinuxBasics/3-8/image-20241229142459323.png)

![image-20241229142550001](/img/docs/jetson/03-LinuxBasics/3-8/image-20241229142550001.png)

### 2.5. Complete the installation

![image-20241229142633937](/img/docs/jetson/03-LinuxBasics/3-8/image-20241229142633937.png)

## 3. Use VSCode

### 3.1. Basic use

Double-click "Visual Studio Code" icon to open the application:

![image-20241229142704122](/img/docs/jetson/03-LinuxBasics/3-8/image-20241229142704122.png)

![image-20241229142818516](/img/docs/jetson/03-LinuxBasics/3-8/image-20241229142818516.png)

### 3.2, Extended use

#### 3.2.1, Basic extension

##### Python

Search for "python" in the extended search bar, and select Python to install:

![image-20250103224402098](/img/docs/jetson/03-LinuxBasics/3-8/image-20250103224402098.png)

##### C/C++

Search for "c/c++" in the extended search bar, and select C/C++, C/C++ Extension Pack to install:

![image-20250103224833396](/img/docs/jetson/03-LinuxBasics/3-8/image-20250103224833396.png)

##### Material Icon Theme

Search for "material icon theme" in the expanded search bar, and select Material Icon Theme to install:

![image-20250103225155546](/img/docs/jetson/03-LinuxBasics/3-8/image-20250103225155546.png)

![image-20250103225255205](/img/docs/jetson/03-LinuxBasics/3-8/image-20250103225255205.png)

#### 3.2.2, Remote-SSH

##### ssh installation

Search for "ssh" in the expanded search bar , select Remote-SSH to install:

![image-20250103214934768](/img/docs/jetson/03-LinuxBasics/3-8/image-20250103214934768.png)

##### ssh usage

After Remote-SSH is installed, you can use it directly through the status bar on the left!

```text
Different users may have inconsistent path usernames in the configuration file, and there is no impression
```


Configure remote: modify the configuration file

![image-20250103215854788](/img/docs/jetson/03-LinuxBasics/3-8/image-20250103215854788.png)

Configure remote: modify the configuration file

Fill in the device information that needs to be remote, the remote device needs to configure the SSH service and support SSH remote; after filling in, use the Ctrl+S shortcut key to save, and the corresponding SSH remote device will be automatically added on the left.

```text
# Read more about SSH config files: https://linux.die.net/man/5/ssh_config
Host MyComputer # Remote device alias
HostName 192.168.66.152 # Remote device IP
User GO # Remote device user name
```

![image-20250103221210749](/img/docs/jetson/03-LinuxBasics/3-8/image-20250103221210749.png)

Remote

Select the remote device for SSH remote:

![image-20250103221608414](/img/docs/jetson/03-LinuxBasics/3-8/image-20250103221608414.png)

Select the remote device platform: fill in according to the actual platform

![image-20250103221642634](/img/docs/jetson/03-LinuxBasics/3-8/image-20250103221642634.png)

![image-20250103222730231](/img/docs/jetson/03-LinuxBasics/3-8/image-20250103222730231.png)

Fill in the password: Enter to confirm after filling in the password

![image-20250103222811721](/img/docs/jetson/03-LinuxBasics/3-8/image-20250103222811721.png)

Remote use

After the remote is successful, there is a prompt in the lower left corner; we can open the folder of the remote device through VSCode

![image-20250103223006694](/img/docs/jetson/03-LinuxBasics/3-8/image-20250103223006694.png)

![image-20250103223939977](/img/docs/jetson/03-LinuxBasics/3-8/image-20250103223939977.png)

Enter the password again:

![image-20250103224011715](/img/docs/jetson/03-LinuxBasics/3-8/image-20250103224011715.png)

![image-20250103225421540](/img/docs/jetson/03-LinuxBasics/3-8/image-20250103225421540.png)
