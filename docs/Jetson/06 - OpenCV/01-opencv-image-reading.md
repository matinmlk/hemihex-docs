---
sidebar_position: 6
title: Image Reading with OpenCV
---

# Image Reading with OpenCV

**OpenCV (Open Source Computer Vision Library)** is an open-source
computer vision and machine learning library widely used in:

-   Image processing\
-   Video processing\
-   Machine vision\
-   Artificial intelligence

This document explains both the **concepts** and a **practical Python
example** for reading images using OpenCV.

------------------------------------------------------------------------

## 1. Implementation Principle

OpenCV provides simple APIs for image input/output:

-   `cv2.imread()` --- reads an image file from disk\
-   `cv2.imshow()` --- displays the image in a window

The basic workflow is:

1.  Load the image from a file path\
2.  Check whether the image is loaded successfully\
3.  Display the image\
4.  Wait for user input and close the window

------------------------------------------------------------------------

## 2. Implementation Effect

Navigate to the OpenCV working directory:

``` bash
cd ~/opencv
```

Run the image reading script:

``` bash
python3 01.image_read.py
```

::: note
Select the image display window and press **`q`** to exit the program.
:::

![Image Display Result](/img/docs/jetson/06-OpenCV/6-1/image-20250106121841781.png)

------------------------------------------------------------------------

## 3. Python Example Code

``` python
import cv2

def read_image(file_path):
    image = cv2.imread(file_path)

    if image is None:
        print("Error: Unable to open image file.")
    else:
        cv2.imshow('Image Preview', image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

read_image('/home/jetson/opencv/images/hemihex_logo.png')
```

------------------------------------------------------------------------

## 4. Code Explanation

-   `cv2.imread(file_path)`\
    Loads the image from disk. Returns `None` if the file cannot be
    read.

-   `cv2.imshow()`\
    Opens a window and displays the image.

-   `cv2.waitKey(0)`\
    Waits indefinitely for a key press.

-   `cv2.destroyAllWindows()`\
    Closes all OpenCV windows.

------------------------------------------------------------------------

Maintained by **HemiHex** for computer vision and OpenCV learning
workflows.
