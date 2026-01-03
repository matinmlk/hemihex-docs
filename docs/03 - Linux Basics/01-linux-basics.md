---
title: Linux Basics
sidebar_position: 0
---

# Linux Basics

Linux Basics 1. Terminal 1.1. Open the terminal 1.2. Basic commands 1.3. Shortcut keys 2. Text editor 2.1. Gedit (Easy) 2.2, Nano (Medium) 2.3, Vi/Vim (difficult)

## 1. Terminal

Terminal is a command line interface used to interact with the operating system.

### 1.1. Open the terminal

In Ubuntu, you can open it by using the shortcut keys Ctrl + Alt + T or by finding the terminal in the application menu:

![image-20250104214102250](/img/docs/jetson/03-Linux Basics/3-1/image-20250104214102250.png)

![image-20250104214129097](/img/docs/jetson/03-Linux Basics/3-1/image-20250104214129097.png)

### 1.2. Basic commands

View the current directory

Display the full path of the current working directory:

```bash
pwd
```

```bash
pwd
```

List files/directories

List files and subdirectories in the current directory:

```bash
ls
```

```bash
xxxxxxxxxx
```

```bash
ls
```

Create a new directory

Create the File_demo directory:

```bash
mkdir File_demo
```

```bash
xxxxxxxxxx
```

```bash
mkdir File_demo
```

Change directory

Enter File_demo directory:

```bash
cd File_demo
```

```bash
xxxxxxxxxx
```

```bash
cd File_demo
```

Return to parent directory

```bash
cd ..
```

```bash
xxxxxxxxxx
```

```bash
cd ..
```

Create new file

Create Version.txt file:

```bash
touch Version.txt
```

```bash
xxxxxxxxxx
```

```bash
touch Version.txt
```

Modify file

Add System Information to Version.txt file:

```bash
echo "System Information" >> Version.txt
```

```bash
xxxxxxxxxx
```

```bash
echo "System Information" >> Version.txt
```

View file

View Version.txt file content:

```bash
cat Version.txt
```

```bash
xxxxxxxxxx
```

```bash
cat Version.txt
```

Delete directory

Delete File_demo directory:

```bash
rm -rf File_demo
```

```bash
xxxxxxxxxx
```

```bash
rm -rf File_demo
```

![image-20250104220819889](/img/docs/jetson/03-Linux Basics/3-1/image-20250104220819889.png)

### 1.3. Shortcut keys

Ctrl + C

Terminate the currently running command

Ctrl + Z

Suspend the current process

Tab

Automatically complete the file name or command

## 2. Text editor

### 2.1. Gedit (Easy)

The default text editor in the GNOME desktop environment, with a graphical user interface (GUI).

Open file

```bash
gedit Version.txt
```

```bash
xxxxxxxxxx
```

```bash
gedit Version.txt
```

![image-20250104221546150](/img/docs/jetson/03-Linux Basics/3-1/image-20250104221546150.png)

### 2.2, Nano (Medium)

Simple and easy-to-use command line text editor, suitable for beginners.

Install

```bash
sudo apt update
sudo apt install nano -y
```

```bash
xxxxxxxxxx
```

```bash
sudo apt update
```

```bash
sudo apt install nano -y
```

Open file

```bash
nano Version.txt
```

```bash
xxxxxxxxxx
```

```bash
nano Version.txt
```

Ctrl + X : Exit (if there are unsaved changes, you will be prompted to save)

Ctrl + U : Paste clipboard contents

Ctrl + W : Search text

![image-20250104225215119](/img/docs/jetson/03-Linux Basics/3-1/image-20250104225215119.png)

### 2.3, Vi/Vim (difficult)

Vim is an enhanced version of Vi editor, suitable for almost all Unix and Linux systems.

Open file

```bash
vi Version.txt
```

```bash
xxxxxxxxxx
```

```bash
vi Version.txt
```

Mode

Command mode: default state

Insert mode: press i to enter, press ESC to exit command mode

Last line mode: enter : in command mode, press ESC to exit command mode

Save/Exit

Last line mode

:w : Save file

:q : Exit

:wq : Save and exit

:q! : Force exit without saving

![image-20250104225248056](/img/docs/jetson/03-Linux Basics/3-1/image-20250104225248056.png)
