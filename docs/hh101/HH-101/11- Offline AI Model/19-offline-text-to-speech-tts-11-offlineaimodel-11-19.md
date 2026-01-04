---
title: Offline Text to Speech (TTS)
sidebar_position: 0
---

# 2.Offline text to speech (TTS)

## 1. Concept Introduction

### 1.1 What is "TTS"?

TTS technology converts written text into human-readable speech output. It enables computers to "read" text aloud and is widely used in a variety of fields, including accessible reading, intelligent assistants, navigation systems, and educational software. Through TTS, users can hear natural, fluent machine-generated human voices, greatly improving the convenience and flexibility of information acquisition.

### 1.2 Overview of Implementation Principles

The implementation of a TTS system primarily involves the following key steps and technologies:

#### 1. Text Analysis

#### 2. Language Processing

#### 3. Speech Synthesis

#### 4. Sound Waveform Generation

With the advancement of artificial intelligence and machine learning technologies, especially the application of deep learning, TTS systems have not only significantly improved in accuracy but also made significant progress in naturalness and emotional expression, making machine-generated speech increasingly similar to human voices.

### 2. Code Analysis

### Key Code

#### 1. TTS Initialization and Invocation ( largemodel/largemodel/model_service.py )

```python
xxxxxxxxxx
# From largemodel/largemodel/model_service.py
class
LargeModelService
(
Node
):
def
__init__
(
self
):
# ...
self
.
system_sound_init
()
# ...
​
def
init_param_config
(
self
):
# ...
self
.
declare_parameter
(
'useolinetts'
,
False
)
self
.
useolinetts
=
self
.
get_parameter
(
'useolinetts'
).
get_parameter_value
().
bool_value
if
self
.
useolinetts
:
self
.
tts_out_path
=
os
.
path
.
join
(
self
.
pkg_path
,
"resources_file"
,
"tts_output.mp3"
)
else
:
self
.
tts_out_path
=
os
.
path
.
join
(
self
.
pkg_path
,
"resources_file"
,
"tts_output.wav"
)
​
def
system_sound_init
(
self
):
"""Initialize TTS system"""
model_type
=
"oline"
if
self
.
useolinetts
else
"local"
self
.
model_client
.
tts_model_init
(
model_type
,
self
.
language
)
self
.
get_logger
().
info
(
f'TTS initialized with {model_type} model'
)
​
def
_safe_play_audio
(
self
,
text_to_speak
:
str
):
"""
Synthesizes and plays all non-empty messages only in non-text chat mode.
"""
if
not
self
.
text_chat_mode
and
text_to_speak
:
try
:
self
.
model_client
.
voice_synthesis
(
text_to_speak
,
self
.
tts_out_path
)
self
.
play_audio_async
(
self
.
tts_out_path
)
except
Exception
as
e
:
self
.
get_logger
().
error
(
f"Safe audio playback failed: {e}"
)
```

```python
xxxxxxxxxx
```

```python
# From largemodel/largemodel/model_service.py
```

```python
class
LargeModelService
(
Node
):
```

```python
def
__init__
(
self
):
```

```python
# ...
```

```python
self
.
system_sound_init
()
```

```python
# ...
```

```python
​
```

```python
def
init_param_config
(
self
):
```

```python
# ...
```

```python
self
.
declare_parameter
(
'useolinetts'
,
False
)
```

```python
self
.
useolinetts
=
self
.
get_parameter
(
'useolinetts'
).
get_parameter_value
().
bool_value
```

```python
if
self
.
useolinetts
:
```

```python
self
.
tts_out_path
=
os
.
path
.
join
(
self
.
pkg_path
,
"resources_file"
,
"tts_output.mp3"
)
```

```python
else
:
```

```python
self
.
tts_out_path
=
os
.
path
.
join
(
self
.
pkg_path
,
"resources_file"
,
"tts_output.wav"
)
```

```python
​
```

```python
def
system_sound_init
(
self
):
```

```python
"""Initialize TTS system"""
```

```python
model_type
=
"oline"
if
self
.
useolinetts
else
"local"
```

```python
self
.
model_client
.
tts_model_init
(
model_type
,
self
.
language
)
```

```python
self
.
get_logger
().
info
(
f'TTS initialized with {model_type} model'
)
```

```python
​
```

```python
def
_safe_play_audio
(
self
,
text_to_speak
:
str
):
```

```python
"""
```

```python
Synthesizes and plays all non-empty messages only in non-text chat mode.
```

```python
"""
```

```python
if
not
self
.
text_chat_mode
and
text_to_speak
:
```

```python
try
:
```

```python
self
.
model_client
.
voice_synthesis
(
text_to_speak
,
self
.
tts_out_path
)
```

```python
self
.
play_audio_async
(
self
.
tts_out_path
)
```

```python
except
Exception
as
e
:
```

```python
self
.
get_logger
().
error
(
f"Safe audio playback failed: {e}"
)
```

#### 2. TTS backend implementation ( largemodel/utils/large_model_interface.py )

```python
xxxxxxxxxx
# From largemodel/utils/large_model_interface.py
class
model_interface
:
# ...
def
tts_model_init
(
self
,
model_type
=
'oline'
,
language
=
'zh'
):
if
model_type
==
'oline'
:
if
self
.
tts_supplier
==
'baidu'
:
self
.
token
=
self
.
fetch_token
()
self
.
model_type
=
'oline'
elif
model_type
==
'local'
:
self
.
model_type
=
'local'
if
language
==
'zh'
:
tts_model
=
self
.
zh_tts_model
tts_json
=
self
.
zh_tts_json
elif
language
==
'en'
:
tts_model
=
self
.
en_tts_model
tts_json
=
self
.
en_tts_json
self
.
synthesizer
=
piper
.
PiperVoice
.
load
(
tts_model
,
config_path
=
tts_json
,
use_cuda
=
False
)
​
def
voice_synthesis
(
self
,
text
,
path
):
if
self
.
model_type
==
'oline'
:
if
self
.
tts_supplier
==
'baidu'
:
# ... (Baidu TTS implementation)
pass
elif
self
.
tts_supplier
==
'aliyun'
:
# ... (Aliyun TTS implementation)
pass
elif
self
.
model_type
==
'local'
:
with
wave
.
open
(
path
,
'wb'
)
as
wav_file
:
wav_file
.
setnchannels
(
1
)
wav_file
.
setsampwidth
(
2
)
wav_file
.
setframerate
(
self
.
synthesizer
.
config
.
sample_rate
)
self
.
synthesizer
.
synthesize
(
text
,
wav_file
)
```

```python
xxxxxxxxxx
```

```python
# From largemodel/utils/large_model_interface.py
```

```python
class
model_interface
:
```

```python
# ...
```

```python
def
tts_model_init
(
self
,
model_type
=
'oline'
,
language
=
'zh'
):
```

```python
if
model_type
==
'oline'
:
```

```python
if
self
.
tts_supplier
==
'baidu'
:
```

```python
self
.
token
=
self
.
fetch_token
()
```

```python
self
.
model_type
=
'oline'
```

```python
elif
model_type
==
'local'
:
```

```python
self
.
model_type
=
'local'
```

```python
if
language
==
'zh'
:
```

```python
tts_model
=
self
.
zh_tts_model
```

```python
tts_json
=
self
.
zh_tts_json
```

```python
elif
language
==
'en'
:
```

```python
tts_model
=
self
.
en_tts_model
```

```python
tts_json
=
self
.
en_tts_json
```

```python
self
.
synthesizer
=
piper
.
PiperVoice
.
load
(
tts_model
,
config_path
=
tts_json
,
use_cuda
=
False
)
```

```python
​
```

```python
def
voice_synthesis
(
self
,
text
,
path
):
```

```python
if
self
.
model_type
==
'oline'
:
```

```python
if
self
.
tts_supplier
==
'baidu'
:
```

```python
# ... (Baidu TTS implementation)
```

```python
pass
```

```python
elif
self
.
tts_supplier
==
'aliyun'
:
```

```python
# ... (Aliyun TTS implementation)
```

```python
pass
```

```python
elif
self
.
model_type
==
'local'
:
```

```python
with
wave
.
open
(
path
,
'wb'
)
as
wav_file
:
```

```python
wav_file
.
setnchannels
(
1
)
```

```python
wav_file
.
setsampwidth
(
2
)
```

```python
wav_file
.
setframerate
(
self
.
synthesizer
.
config
.
sample_rate
)
```

```python
self
.
synthesizer
.
synthesize
(
text
,
wav_file
)
```

### Code Analysis

The text-to-speech (TTS) function is invoked by the LargeModelService node and implemented by the model_interface class. Its design uses parameter configuration to switch between different backend services.

## 3. Practical Operations

### 3.1 Configuring Offline TTS

To enable offline TTS, you need to correctly configure HemiHex.yaml and large_model_interface.yaml and ensure that the local model is correctly placed.

Open the main configuration file :

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

Modify/confirm the following key configurations:

```python
xxxxxxxxxx
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
regional_setting
:
"international"
```

```python
xxxxxxxxxx
```

```python
model_service
:
#Model server node parameters
```

```python
ros__parameters
:
```

```python
language
:
'en'
#Large Model Interface Language
```

```python
useolinetts
:
False
#Whether to use online speech synthesis (True to use online, False to use offline)
```

```python
regional_setting
:
"international"
```

Make sure useolinetts is set to False to use the local model.

Select "zh" for Chinese and "en" for English.

Open the model interface configuration file :

```python
xxxxxxxxxx
vim
~/yahboom_ws/src/largemodel/config/large_model_interface.yaml
```

```python
xxxxxxxxxx
```

```python
vim
~/yahboom_ws/src/largemodel/config/large_model_interface.yaml
```

Confirm offline model path :

```python
xxxxxxxxxx
# large_model_interface.yaml
## 离线语音合成 (Offline TTS)
​
# English TTS model
en_tts_model
:
"/home/jetson/yahboom_ws/src/largemodel/MODELS/tts/en/en_US-libritts-high.onnx"
en_tts_json
:
"/home/jetson/yahboom_ws/src/largemodel/MODELS/tts/en/en_US-libritts-high.onnx.json"
```

```python
xxxxxxxxxx
```

```python
# large_model_interface.yaml
```

```python
## 离线语音合成 (Offline TTS)
```

```python
​
```

```python
# English TTS model
```

```python
en_tts_model
:
"/home/jetson/yahboom_ws/src/largemodel/MODELS/tts/en/en_US-libritts-high.onnx"
```

```python
en_tts_json
:
"/home/jetson/yahboom_ws/src/largemodel/MODELS/tts/en/en_US-libritts-high.onnx.json"
```

### 3.2 Start and test the functionality

Start the TTS node : Run the following command:

```python
xxxxxxxxxx
ros2 launch largemodel tts_only.launch.py
```

```python
xxxxxxxxxx
```

```python
ros2 launch largemodel tts_only.launch.py
```

![image-20250807165129463](/img/docs/jetson/11-OfflineAIModel/11-19/1.png)

Send the text to be synthesized : Open a new terminal and run the following command to publish a voice message:

```python
xxxxxxxxxx
ros2 topic pub
--once
/tts_text_input std_msgs/msg/String
'{data: "Speech synthesis test successful"}'
```

```python
xxxxxxxxxx
```

```python
ros2 topic pub
--once
/tts_text_input std_msgs/msg/String
'{data: "Speech synthesis test successful"}'
```

Test : If everything went well, you should hear the robot say "Speech synthesis test successful" in a synthesized voice through your speakers.

## 4. Common Problems and Solutions

### 4.1 Playback Issues

#### Issue 1: The program runs normally without errors, but no sound is heard.

Solution :

Check the audio output : Confirm that your system audio output device is selected correctly and the volume is not muted. Try playing a standard music file to test the hardware.

In the Sound settings, set the Balance to the middle position.

![image-20250807155119783](/img/docs/jetson/11-OfflineAIModel/11-19/image-20250807155119783.png)
