---
sidebar_position: 17
title: 13 - Add Circular Shape with OpenCV
---

# Add Circular Shape with OpenCV

This section explains how to **draw a circular shape on an image** using
OpenCV in Python.

Drawing circles is commonly used for: - Marking points of interest -
Visualizing detections - Debugging and annotation in computer vision
pipelines

------------------------------------------------------------------------

## 1. Implementation Principle

OpenCV provides the `cv2.circle()` function to draw a circle directly on
an image.

``` python
cv2.circle(image, center, radius, color, thickness)
```

Where: - `image` is the input image - `center` is the `(x, y)`
coordinate of the circle center - `radius` is the circle radius in
pixels - `color` is the circle color in **BGR** format - `thickness`
defines line thickness (`-1` fills the circle)

------------------------------------------------------------------------

## 2. Implementation Effect

Navigate to the OpenCV working directory:

``` bash
cd ~/opencv
```

Run the circle drawing script:

``` bash
python3 13.image_draw_circle.py
```

::: note
Select the image window and press **`q`** to exit the program.
:::

![Circular Shape
Result](/img/docs/jetson/06-OpenCV/6-13/image-20250106182835811.png)

------------------------------------------------------------------------

## 3. Implementation Code

``` python
import cv2

def draw_circle(input_path, output_path, center, radius, color, thickness):
    image = cv2.imread(input_path)

    if image is None:
        print("Error: Unable to open image file.")
        return

    cv2.circle(image, center, radius, color, thickness)

    if cv2.imwrite(output_path, image):
        print(f"Image saved to {output_path}")
        cv2.imshow('Image Preview', cv2.imread(output_path))
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    else:
        print("Error: Unable to save image file.")

draw_circle(
    '/home/jetson/opencv/images/hemihex_logo.png',
    '/home/jetson/opencv/images/hemihex_logo_circle.png',
    (88, 50),
    30,
    (0, 0, 255),
    2
)
```

------------------------------------------------------------------------

## 4. Code Explanation

-   `cv2.imread()` loads the source image\
-   `cv2.circle()` draws a circle using the given parameters\
-   `cv2.imwrite()` saves the annotated image\
-   Display functions preview the result

------------------------------------------------------------------------

Maintained by **HemiHex** for OpenCV-based image processing workflows.
