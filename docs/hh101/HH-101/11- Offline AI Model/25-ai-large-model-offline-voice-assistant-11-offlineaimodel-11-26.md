---
title: AI Large Model Offline Voice Assistant
sidebar_position: 0
---

# 10.AI large model offline voice assistant

## 1. Offline Voice Configuration

Note: Due to performance limitations, this example cannot be run on the Jetson Orin Nano 4GB. To experience this feature, please refer to the corresponding section in [Online Large Model (Voice Interaction)]

Before setting up automatic startup, we must ensure that the program itself can operate independently in an offline state. This requires modifying the configuration file.

Locate the Configuration File : In your project code, find and open the configuration file:

config/HemiHex.yaml

Modify Configuration Parameters : Please check the following parameters in the file and ensure that their values ​​match those shown below. If the parameters do not exist, add them.

```yaml

asr
:
#Voice node parameters
ros__parameters
:
VAD_MODE
:
2
#VAD sensitivity
sample_rate
:
16000
#ASR audio sampling rate
frame_duration_ms
:
30
#VAD frame size in milliseconds
use_oline_asr
:
False
#Whether to use online ASR recognition (True for online, False for offline)
mic_serial_port
:
"/dev/ttyUSB0"
#Microphone serial port alias
mic_index
:
0
#Microphone index
language
:
'en'
#ASR language
regional_setting
:
"international"
#international：International Edition  China：China internal version
​
model_service
:
#Model server node parameters
ros__parameters
:
language
:
'en'
#Large Model Interface Language
useolinetts
:
False
#Whether to use online speech synthesis (True to use online, False to use offline)
​
# Large model configuration
# llm_platform: 'ollama'              # Optional platforms: 'ollama', 'tongyi', 'spark', 'qianfan', 'openrouter'
llm_platform
:
'ollama'
# Currently selected large model platform
regional_setting
:
"international"
​
```


```yaml
asr
:
#Voice node parameters
```

```yaml
ros__parameters
:
```

```yaml
VAD_MODE
:
2
#VAD sensitivity
```

```yaml
sample_rate
:
16000
#ASR audio sampling rate
```

```yaml
frame_duration_ms
:
30
#VAD frame size in milliseconds
```

```yaml
use_oline_asr
:
False
#Whether to use online ASR recognition (True for online, False for offline)
```

```yaml
mic_serial_port
:
"/dev/ttyUSB0"
#Microphone serial port alias
```

```yaml
mic_index
:
0
#Microphone index
```

```yaml
language
:
'en'
#ASR language
```

```yaml
regional_setting
:
"international"
#international：International Edition  China：China internal version
```

```bash
​
```

```yaml
model_service
:
#Model server node parameters
```

```yaml
ros__parameters
:
```

```yaml
language
:
'en'
#Large Model Interface Language
```

```yaml
useolinetts
:
False
#Whether to use online speech synthesis (True to use online, False to use offline)
```

```bash
​
```

```bash
# Large model configuration
```

```bash
# llm_platform: 'ollama'              # Optional platforms: 'ollama', 'tongyi', 'spark', 'qianfan', 'openrouter'
```

```yaml
llm_platform
:
'ollama'
# Currently selected large model platform
```

```yaml
regional_setting
:
"international"
```

```bash
​
```

This setting will make everything offline.

Save the file and recompile the project to apply the changes:

```bash

cd
~/yahboom_ws
colcon build
source
install/setup.bash
```


```bash
cd
~/yahboom_ws
```

```bash
colcon build
```

```bash
source
install/setup.bash
```

After completing this step, the program is already a purely offline voice service.

## 2. Create a Startup Service (Systemd)

Now, we will create a systemd service to automatically run largemodel_control.launch.py at system startup.

### 2.1 Create a Startup Script

To ensure systemd can properly load the ROS2 environment, it's best to create a simple bash script to encapsulate our startup commands.

Create the script file :

In the directory ( ~/yahboom_ws/src/largemodel/ ), create a file named start_largemodel.sh .

```bash

vim
~/yahboom_ws/src/largemodel/start_largemodel.sh
```


```bash
vim
~/yahboom_ws/src/largemodel/start_largemodel.sh
```

Write script content : Copy and paste the following content into the script file.

```bash

#!/bin/bash
​
# Source ROS2 Humble environment
source
/opt/ros/humble/setup.bash
​
# SOurce HemiHex workspace environment
source
/home/jetson/yahboom_ws/install/setup.bash
​
#Start the largemodel control script
ros2 launch largemodel largemodel_control.launch.py
```


```bash
#!/bin/bash
```

```bash
​
```

```bash
# Source ROS2 Humble environment
```

```bash
source
/opt/ros/humble/setup.bash
```

```bash
​
```

```bash
# SOurce HemiHex workspace environment
```

```bash
source
/home/jetson/yahboom_ws/install/setup.bash
```

```bash
​
```

```bash
#Start the largemodel control script
```

```bash
ros2 launch largemodel largemodel_control.launch.py
```

IMPORTANT : Please make sure to replace /home/sunrise/ in the script with your own user home directory path.

Save and exit

Give the script execution permissions :

```bash

chmod
+
x ~/yahboom_ws/src/largemodel/start_largemodel.sh
```


```bash
chmod
+
x ~/yahboom_ws/src/largemodel/start_largemodel.sh
```

### 2.2Creating the Systemd Service File

This is the most crucial step. We'll tell the system that we have a new service to manage.

Create service file : You will need sudo privileges to create this file.

```bash

sudo
vim
/etc/systemd/system/largemodel.service
```


```bash
sudo
vim
/etc/systemd/system/largemodel.service
```

Write service configuration : Copy and paste the following content into the service file.

```yaml

[Unit]
Description
=
Robot Service
After
=
network.target sound.target graphical.target multi-user.target
Wants
=
network.target sound.target graphical.target multi-user.target
​
[Service]
Type
=
simple
User
=
sunrise
Group
=
sunrise
Environment
=
DISPLAY
=:
0
Environment
=
XDG_RUNTIME_DIR
=
/run/user/1000
Environment
=
PULSE_SERVER
=
unix
:
/run/user/1000/pulse/native
SupplementaryGroups
=
audio video
ExecStartPre
=
/bin/sleep 10
ExecStart
=
/home/sunrise/yahboom_ws/src/largemodel/start_largemodel.sh
Restart
=
on-failure
StandardOutput
=
journal
StandardError
=
journal
​
[Install]
WantedBy
=
multi-user.target
```


```bash
[Unit]
```

```bash
Description
=
Robot Service
```

```bash
After
=
network.target sound.target graphical.target multi-user.target
```

```bash
Wants
=
network.target sound.target graphical.target multi-user.target
```

```bash
​
```

```bash
[Service]
```

```bash
Type
=
simple
```

```bash
User
=
sunrise
```

```bash
Group
=
sunrise
```

```yaml
Environment
=
DISPLAY
=:
0
```

```bash
Environment
=
XDG_RUNTIME_DIR
=
/run/user/1000
```

```yaml
Environment
=
PULSE_SERVER
=
unix
:
/run/user/1000/pulse/native
```

```bash
SupplementaryGroups
=
audio video
```

```bash
ExecStartPre
=
/bin/sleep 10
```

```bash
ExecStart
=
/home/sunrise/yahboom_ws/src/largemodel/start_largemodel.sh
```

```bash
Restart
=
on-failure
```

```bash
StandardOutput
=
journal
```

```bash
StandardError
=
journal
```

```bash
​
```

```bash
[Install]
```

```bash
WantedBy
=
multi-user.target
```

Save and exit .

### 2.3 Managing and Debugging Services

Now that your service has been created, we need to have systemd load it and configure it to start automatically at boot.

Reload the systemd daemon so that it reads our newly created service file:

```bash

sudo
systemctl daemon-reload
```


```bash
sudo
systemctl daemon-reload
```

Set the service to start automatically at boot :

```bash

sudo
systemctl enable largemodel.service
```


```bash
sudo
systemctl enable largemodel.service
```

Start the service immediately :

```bash

sudo
systemctl
start
largemodel.service
```


```bash
sudo
systemctl
start
largemodel.service
```

Check service status : This is the most important command to verify that the service is running successfully.

```bash

sudo
systemctl status largemodel.service
```


```bash
sudo
systemctl status largemodel.service
```

View service log (required for debugging) : If the service fails to start, you can use the following command to view all real-time logs generated by the ros2 launch command, which is crucial for locating the problem.

```bash

journalctl
-u
largemodel.service
-f
```


```bash
journalctl
-u
largemodel.service
-f
```

After completing all the above steps, the purely offline largemodel voice service will be automatically started every time you turn on your phone.
