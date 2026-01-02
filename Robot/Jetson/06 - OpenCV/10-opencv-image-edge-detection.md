---
sidebar_position: 14
title: Image Edge Detection with OpenCV
---

# Image Edge Detection with OpenCV

This section explains how to **detect edges in an image** using OpenCV's
Canny edge detection algorithm.

Edge detection is commonly used for: - Feature extraction - Shape
detection - Object boundary analysis - Preprocessing for vision
pipelines

------------------------------------------------------------------------

## 1. Implementation Principle

OpenCV provides the `cv2.Canny()` function for edge detection.

``` python
cv2.Canny(image, threshold1, threshold2)
```

Where:

-   `image` is a **grayscale image**
-   `threshold1` is the lower hysteresis threshold
-   `threshold2` is the upper hysteresis threshold

The output is a binary image highlighting strong edges.

------------------------------------------------------------------------

## 2. Implementation Effect

Navigate to the OpenCV working directory:

``` bash
cd ~/opencv
```

Run the edge detection script:

``` bash
python3 10.image_edge.py
```

::: note
Select the image window and press **`q`** to exit the program.
:::

![Image Edge Detection
Result](/img/docs/jetson/06-OpenCV/6-10/image-20250106161749169.png)

------------------------------------------------------------------------

## 3. Implementation Code

``` python
import cv2

def edge_detection(input_path, output_path, threshold1, threshold2):
    image = cv2.imread(input_path, cv2.IMREAD_GRAYSCALE)

    if image is None:
        print("Error: Unable to open image file.")
        return

    edges = cv2.Canny(image, threshold1, threshold2)

    if cv2.imwrite(output_path, edges):
        print(f"Image saved to {output_path}")
        cv2.imshow('Image Preview', cv2.imread(output_path))
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    else:
        print("Error: Unable to save image file.")

edge_detection(
    '/home/jetson/opencv/images/hemihex_logo.png',
    '/home/jetson/opencv/images/hemihex_logo_edge.png',
    100,
    200
)
```

------------------------------------------------------------------------

## 4. Code Explanation

-   `cv2.imread(..., cv2.IMREAD_GRAYSCALE)`\
    Loads the image in grayscale mode (required for Canny).

-   `cv2.Canny()`\
    Detects edges based on gradient intensity.

-   `cv2.imwrite()`\
    Saves the edge-detected image to disk.

-   `cv2.imshow()` / `cv2.waitKey()` / `cv2.destroyAllWindows()`\
    Displays the result and closes the window.

------------------------------------------------------------------------

## Summary

-   Edge detection highlights object boundaries
-   Canny is one of the most widely used edge detectors
-   Requires grayscale input
-   Threshold values control sensitivity

------------------------------------------------------------------------

Maintained by **HemiHex** for OpenCV-based image processing workflows.
