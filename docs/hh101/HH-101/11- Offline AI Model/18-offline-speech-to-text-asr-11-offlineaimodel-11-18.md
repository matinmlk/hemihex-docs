---
title: Offline Speech to Text (ASR)
sidebar_position: 0
---

# 1.Offline speech to text (ASR)

## 1. Introduction

### 1.1 What is "ASR"?

ASR (Automatic Speech Recognition) is a technology that converts human speech signals into text. It is widely used in intelligent assistants, voice command control, telephone customer service automation, and real-time subtitle generation. The goal of ASR is to enable machines to "understand" human speech and convert it into a form that computers can process and understand.

### 1.2 Implementation Principles

The implementation of an ASR system relies primarily on the following key technical components:

#### 1. Acoustic Model

#### 2. Language Model

#### 3. Pronunciation Dictionary

#### 4. Decoder

#### 5. End-to-End ASR

In general, modern ASR systems achieve efficient and accurate speech-to-text conversion by combining the aforementioned components and leveraging large datasets and powerful computing resources for training. With technological advances, the performance of ASR systems continues to improve, and their application scenarios are becoming increasingly broad.

--

## 2. Code Analysis

### Key Code

#### 1. Speech Processing and Recognition Core ( largemodel/largemodel/asr.py )

```python
xxxxxxxxxx
# From largemodel/largemodel/asr.py
def
kws_handler
(
self
)
->
None
:
if
self
.
stop_event
.
is_set
():
return
​
if
self
.
listen_for_speech
(
self
.
mic_index
):
asr_text
=
self
.
ASR_conversion
(
self
.
user_speechdir
)
# 进行 ASR 转换 / Perform ASR conversion
if
asr_text
==
'error'
:
# 检查 ASR 结果长度是否小于4个字符 / Check if ASR result length is less than 4 characters
self
.
get_logger
().
warn
(
"I still don't understand what you mean. Please try again"
)
playsound
(
self
.
audio_dict
[
self
.
error_response
])
# 错误响应 / Error response
else
:
self
.
get_logger
().
info
(
asr_text
)
self
.
get_logger
().
info
(
"okay😀, let me think for a moment..."
)
self
.
asr_pub_result
(
asr_text
)
# 发布 ASR结果 / Publish ASR result
else
:
return
​
def
ASR_conversion
(
self
,
input_file
:
str
)
->
str
:
if
self
.
use_oline_asr
:
result
=
self
.
modelinterface
.
oline_asr
(
input_file
)
if
result
[
0
] ==
'ok'
and
len
(
result
[
1
])
>
4
:
return
result
[
1
]
else
:
self
.
get_logger
().
error
(
f'ASR Error:{result[1]}'
)
# ASR error.
return
'error'
else
:
result
=
self
.
modelinterface
.
SenseVoiceSmall_ASR
(
input_file
)
if
result
[
0
] ==
'ok'
and
len
(
result
[
1
])
>
4
:
return
result
[
1
]
else
:
self
.
get_logger
().
error
(
f'ASR Error:{result[1]}'
)
# ASR error.
return
'error'
```

```python
xxxxxxxxxx
```

```python
# From largemodel/largemodel/asr.py
```

```python
def
kws_handler
(
self
)
->
None
:
```

```python
if
self
.
stop_event
.
is_set
():
```

```python
return
```

```python
​
```

```python
if
self
.
listen_for_speech
(
self
.
mic_index
):
```

```python
asr_text
=
self
.
ASR_conversion
(
self
.
user_speechdir
)
# 进行 ASR 转换 / Perform ASR conversion
```

```python
if
asr_text
==
'error'
:
# 检查 ASR 结果长度是否小于4个字符 / Check if ASR result length is less than 4 characters
```

```python
self
.
get_logger
().
warn
(
"I still don't understand what you mean. Please try again"
)
```

```python
playsound
(
self
.
audio_dict
[
self
.
error_response
])
# 错误响应 / Error response
```

```python
else
:
```

```python
self
.
get_logger
().
info
(
asr_text
)
```

```python
self
.
get_logger
().
info
(
"okay😀, let me think for a moment..."
)
```

```python
self
.
asr_pub_result
(
asr_text
)
# 发布 ASR结果 / Publish ASR result
```

```python
else
:
```

```python
return
```

```python
​
```

```python
def
ASR_conversion
(
self
,
input_file
:
str
)
->
str
:
```

```python
if
self
.
use_oline_asr
:
```

```python
result
=
self
.
modelinterface
.
oline_asr
(
input_file
)
```

```python
if
result
[
0
] ==
'ok'
and
len
(
result
[
1
])
>
4
:
```

```python
return
result
[
1
]
```

```python
else
:
```

```python
self
.
get_logger
().
error
(
f'ASR Error:{result[1]}'
)
# ASR error.
```

```python
return
'error'
```

```python
else
:
```

```python
result
=
self
.
modelinterface
.
SenseVoiceSmall_ASR
(
input_file
)
```

```python
if
result
[
0
] ==
'ok'
and
len
(
result
[
1
])
>
4
:
```

```python
return
result
[
1
]
```

```python
else
:
```

```python
self
.
get_logger
().
error
(
f'ASR Error:{result[1]}'
)
# ASR error.
```

```python
return
'error'
```

#### 2. VAD smart recording ( largemodel/largemodel/asr.py )

```python
xxxxxxxxxx
# From largemodel/largemodel/asr.py
def
listen_for_speech
(
self
,
mic_index
=
0
):
p
=
pyaudio
.
PyAudio
()
# Create PyAudio instance. / 创建PyAudio实例。
audio_buffer
= []
# Store audio data. / 存储音频数据。
silence_counter
=
0
# Silence counter. / 静音计数器。
MAX_SILENCE_FRAMES
=
90
# 30帧*30ms=900ms静音后停止 / Stop after 900ms of silence (30 frames * 30ms)
speaking
=
False
# Flag indicating speech activity. / 语音活动标志。
frame_counter
=
0
# Frame counter. / 计数器。
stream_kwargs
= {
'format'
:
pyaudio
.
paInt16
,
'channels'
:
1
,
'rate'
:
self
.
sample_rate
,
'input'
:
True
,
'frames_per_buffer'
:
self
.
frame_bytes
,
}
if
mic_index
!
=
0
:
stream_kwargs
[
'input_device_index'
] =
mic_index
​
# Prompt the user to speak via the buzzer. / 通过蜂鸣器提示用户讲话。
self
.
pub_beep
.
publish
(
UInt16
(
data
=
1
))
time
.
sleep
(
0.5
)
self
.
pub_beep
.
publish
(
UInt16
(
data
=
0
))
​
try
:
# Open audio stream. / 打开音频流。
stream
=
p
.
open
(
**
stream_kwargs
)
while
True
:
if
self
.
stop_event
.
is_set
():
return
False
frame
=
stream
.
read
(
self
.
frame_bytes
,
exception_on_overflow
=
False
)
# Read audio data. / 读取音频数据。
is_speech
=
self
.
vad
.
is_speech
(
frame
,
self
.
sample_rate
)
# VAD detection. / VAD检测。
​
if
is_speech
:
# Detected speech activity. / 检测到语音活动。
speaking
=
True
audio_buffer
.
append
(
frame
)
silence_counter
=
0
else
:
if
speaking
:
# Detect silence after speech activity. / 在语音活动后检测静音。
silence_counter
+=
1
audio_buffer
.
append
(
frame
)
# Continue recording buffer. / 持续记录缓冲。
# End recording when silence duration meets the threshold. / 静音持续时间达标时结束录音。
if
silence_counter
>
=
MAX_SILENCE_FRAMES
:
break
frame_counter
+=
1
if
frame_counter
%
2
==
0
:
self
.
get_logger
().
info
(
'1'
if
is_speech
else
'-'
)
# Real-time status display.
finally
:
stream
.
stop_stream
()
stream
.
close
()
p
.
terminate
()
​
# Save valid recording (remove trailing silence). / 保存有效录音（去除尾部静音）。
if
speaking
and
len
(
audio_buffer
)
>
0
:
# Trim the last silent part. / 裁剪最后静音部分。
clean_buffer
=
audio_buffer
[:
-
MAX_SILENCE_FRAMES
]
if
len
(
audio_buffer
)
>
MAX_SILENCE_FRAMES
else
audio_buffer
with
wave
.
open
(
self
.
user_speechdir
,
'wb'
)
as
wf
:
wf
.
setnchannels
(
1
)
wf
.
setsampwidth
(
p
.
get_sample_size
(
pyaudio
.
paInt16
))
wf
.
setframerate
(
self
.
sample_rate
)
wf
.
writeframes
(
b''
.
join
(
clean_buffer
))
return
True
```

```python
xxxxxxxxxx
```

```python
# From largemodel/largemodel/asr.py
```

```python
def
listen_for_speech
(
self
,
mic_index
=
0
):
```

```python
p
=
pyaudio
.
PyAudio
()
# Create PyAudio instance. / 创建PyAudio实例。
```

```python
audio_buffer
= []
# Store audio data. / 存储音频数据。
```

```python
silence_counter
=
0
# Silence counter. / 静音计数器。
```

```python
MAX_SILENCE_FRAMES
=
90
# 30帧*30ms=900ms静音后停止 / Stop after 900ms of silence (30 frames * 30ms)
```

```python
speaking
=
False
# Flag indicating speech activity. / 语音活动标志。
```

```python
frame_counter
=
0
# Frame counter. / 计数器。
```

```python
stream_kwargs
= {
```

```python
'format'
:
pyaudio
.
paInt16
,
```

```python
'channels'
:
1
,
```

```python
'rate'
:
self
.
sample_rate
,
```

```python
'input'
:
True
,
```

```python
'frames_per_buffer'
:
self
.
frame_bytes
,
```

```python
}
```

```python
if
mic_index
!
=
0
:
```

```python
stream_kwargs
[
'input_device_index'
] =
mic_index
```

```python
​
```

```python
# Prompt the user to speak via the buzzer. / 通过蜂鸣器提示用户讲话。
```

```python
self
.
pub_beep
.
publish
(
UInt16
(
data
=
1
))
```

```python
time
.
sleep
(
0.5
)
```

```python
self
.
pub_beep
.
publish
(
UInt16
(
data
=
0
))
```

```python
​
```

```python
try
:
```

```python
# Open audio stream. / 打开音频流。
```

```python
stream
=
p
.
open
(
**
stream_kwargs
)
```

```python
while
True
:
```

```python
if
self
.
stop_event
.
is_set
():
```

```python
return
False
```

```python
frame
=
stream
.
read
(
self
.
frame_bytes
,
exception_on_overflow
=
False
)
# Read audio data. / 读取音频数据。
```

```python
is_speech
=
self
.
vad
.
is_speech
(
frame
,
self
.
sample_rate
)
# VAD detection. / VAD检测。
```

```python
​
```

```python
if
is_speech
:
```

```python
# Detected speech activity. / 检测到语音活动。
```

```python
speaking
=
True
```

```python
audio_buffer
.
append
(
frame
)
```

```python
silence_counter
=
0
```

```python
else
:
```

```python
if
speaking
:
```

```python
# Detect silence after speech activity. / 在语音活动后检测静音。
```

```python
silence_counter
+=
1
```

```python
audio_buffer
.
append
(
frame
)
# Continue recording buffer. / 持续记录缓冲。
```

```python
# End recording when silence duration meets the threshold. / 静音持续时间达标时结束录音。
```

```python
if
silence_counter
>
=
MAX_SILENCE_FRAMES
:
```

```python
break
```

```python
frame_counter
+=
1
```

```python
if
frame_counter
%
2
==
0
:
```

```python
self
.
get_logger
().
info
(
'1'
if
is_speech
else
'-'
)
```

```python
# Real-time status display.
```

```python
finally
:
```

```python
stream
.
stop_stream
()
```

```python
stream
.
close
()
```

```python
p
.
terminate
()
```

```python
​
```

```python
# Save valid recording (remove trailing silence). / 保存有效录音（去除尾部静音）。
```

```python
if
speaking
and
len
(
audio_buffer
)
>
0
:
```

```python
# Trim the last silent part. / 裁剪最后静音部分。
```

```python
clean_buffer
=
audio_buffer
[:
-
MAX_SILENCE_FRAMES
]
if
len
(
audio_buffer
)
>
MAX_SILENCE_FRAMES
else
audio_buffer
```

```python
with
wave
.
open
(
self
.
user_speechdir
,
'wb'
)
as
wf
:
```

```python
wf
.
setnchannels
(
1
)
```

```python
wf
.
setsampwidth
(
p
.
get_sample_size
(
pyaudio
.
paInt16
))
```

```python
wf
.
setframerate
(
self
.
sample_rate
)
```

```python
wf
.
writeframes
(
b''
.
join
(
clean_buffer
))
```

```python
return
True
```

### Code Analysis

ASR (speech-to-text) functionality is provided by the ASRNode node ( asr.py ). This node is responsible for recording, converting, and publishing audio.

## 3. Practical Operations

### 3.1 Configuring Offline ASR

To enable offline ASR, you need to correctly configure the HemiHex.yaml file and ensure that the local model is correctly placed.

Open the configuration file :

```python
xxxxxxxxxx
vim
~/yahboom_ws/src/largemodel/config/HemiHex.yaml
```

```python
xxxxxxxxxx
```

```python
vim
~/yahboom_ws/src/largemodel/config/HemiHex.yaml
```

Modify/confirm the following key configuration :

```python
xxxxxxxxxx
asr
:
#Voice node parameters
ros__parameters
:
# ...
use_oline_asr
:
False
# KEY: Must be set to False to enable offline ASR
mic_serial_port
:
"/dev/ttyUSB0"
# Microphone serial port alias
mic_index
:
0
# Microphone Device Index
language
:
'en'
# asr language, 'zh' or 'en'
regional_setting
:
"international"
```

```python
xxxxxxxxxx
```

```python
asr
:
#Voice node parameters
```

```python
ros__parameters
:
```

```python
# ...
```

```python
use_oline_asr
:
False
# KEY: Must be set to False to enable offline ASR
```

```python
mic_serial_port
:
"/dev/ttyUSB0"
# Microphone serial port alias
```

```python
mic_index
:
0
# Microphone Device Index
```

```python
language
:
'en'
# asr language, 'zh' or 'en'
```

```python
regional_setting
:
"international"
```

Make sure use_oline_asr is set to False to use the local model.

In the terminal, enter ls /dev/ttyUSB* to check if the USB device number assigned to the voice module is USB0. If not, replace the 0 in the configuration file with your desired device number.

Select "zh" for Chinese and "en" for English.

Also, specify the path to the offline model in large_model_interface.yaml .

Open the file in the terminal:

```python
xxxxxxxxxx
vim ~/yahboom_ws/src/largemodel/config/large_model_interface.yaml
```

```python
xxxxxxxxxx
```

```python
vim ~/yahboom_ws/src/largemodel/config/large_model_interface.yaml
```

Find the configuration related to local_asr_model .

```python
xxxxxxxxxx
# large_model_interface.yaml
## 离线语音识别 (Offline ASR)
local_asr_model
:
"/home/jetson/yahboom_ws/src/largemodel/MODELS/asr/SenseVoiceSmall"
# Local ASR model path
```

```python
xxxxxxxxxx
```

```python
# large_model_interface.yaml
```

```python
## 离线语音识别 (Offline ASR)
```

```python
local_asr_model
:
"/home/jetson/yahboom_ws/src/largemodel/MODELS/asr/SenseVoiceSmall"
# Local ASR model path
```

### 3.2 Start and test the functionality

Startup Command :

```python
xxxxxxxxxx
ros2 launch largemodel asr_only.launch.py
```

```python
xxxxxxxxxx
```

```python
ros2 launch largemodel asr_only.launch.py
```

![image-20250807154240917](/img/docs/jetson/11-OfflineAIModel/11-18/1.png)
