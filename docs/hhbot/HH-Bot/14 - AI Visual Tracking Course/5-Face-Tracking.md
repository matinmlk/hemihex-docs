---
title: Face tracking
sidebar_position: 5
---

### 1. Function Introduction

Based on the facial positioning function, combined with the robotic arm to achieve facial tracking function.

Code path：~/jetcobot_ws/src/jetcobot_face_follow/face_tracking.ipynb

### 2. About code

- Import header file

```python
import cv2 as cv
import threading
from time import sleep
import ipywidgets as widgets
from IPython.display import display
from face_follow import face_follow
```

- Create an instance and initialize parameters

```python
# Create an instance
follow = face_follow()
# Initialization mode
model = 'General'
```

- Create Control

```python
button_layout = widgets.Layout(width='250px', height='50px', align_self='center')
output = widgets.Output()
# exit button
exit_button = widgets.Button(description='Exit', button_style='danger', layout=button_layout)
# Image widget
imgbox = widgets.Image(format='jpg', height=480, width=640, layout=widgets.Layout(align_self='center'))
# spatial distribution
controls_box = widgets.VBox([imgbox, exit_button], layout=widgets.Layout(align_self='center'))
# ['auto', 'flex-start', 'flex-end', 'center', 'baseline', 'stretch', 'inherit', 'initial', 'unset']
```

- Mode switching

```python
def exit_button_Callback(value):
global model
model = 'Exit'
#     with output: print(model)
exit_button.on_click(exit_button_Callback)
```

- Main program

```python
def camera():
global model
# Open camera
capture = cv.VideoCapture(0)
while capture.isOpened():
try:
_, img = capture.read()
img = cv.resize(img, (640, 480))
img = follow.follow_function(img)
if model == 'Exit':
cv.destroyAllWindows()
capture.release()
break
imgbox.value = cv.imencode('.jpg', img)[1].tobytes()
except KeyboardInterrupt:capture.release()
```

- Start

```python
display(controls_box,output)
threading.Thread(target=camera, ).start()
```

### 3. Run program

Click the run button on the jupyterlab toolbar, run the entire program, and then drag it to the bottom.

![202309010001](/img/docs/hhbot/14-ai-visual-tracking-course/14-5/202309010001.png)

You can see the camera image, and when you place the face into the camera image, the robotic arm will move along with the face.

Note: When moving the face, the speed should not be too fast, otherwise the robotic arm may not be able to keep up due to the movement being too fast.

![1697535412026_7AF07D3E-1E1D-4121-AF16-24D983DB696B](/img/docs/hhbot/14-ai-visual-tracking-course/14-5/1697535412026_7AF07D3E-1E1D-4121-AF16-24D983DB696B.png)
