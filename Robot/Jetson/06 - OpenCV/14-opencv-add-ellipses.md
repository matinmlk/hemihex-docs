---
sidebar_position: 18
title: Add Ellipses with OpenCV
---

# Add Ellipses with OpenCV

This section explains how to **draw ellipses on an image** using OpenCV
in Python.

Ellipses are commonly used for: - Highlighting rounded regions of
interest - Visual annotations - Debugging and visualization in computer
vision workflows

------------------------------------------------------------------------

## 1. Implementation Principle

Use the `cv2.ellipse()` function to draw an ellipse on an image.

``` python
cv2.ellipse(image, center, axes, angle, start_angle, end_angle, color, thickness)
```

Where: - `image` is the input image - `center` is the `(x, y)`
coordinate of the ellipse center - `axes` defines the length of the
major and minor axes - `angle` is the rotation angle of the ellipse -
`start_angle` and `end_angle` define the arc range - `color` is
specified in **BGR** format - `thickness` controls line thickness (`-1`
fills the ellipse)

------------------------------------------------------------------------

## 2. Implementation Effect

Navigate to the OpenCV working directory:

``` bash
cd ~/opencv
```

Run the ellipse drawing script:

``` bash
python3 14.image_draw_ellipse.py
```

::: note
Select the image window and press **`q`** to exit the program.
:::

![Ellipse Drawing
Result](/img/docs/jetson/06-OpenCV/6-14/image-20250106172939112.png)

------------------------------------------------------------------------

## 3. Implementation Code

``` python
import cv2

def draw_ellipse(input_path, output_path, center, axes, angle, start_angle, end_angle, color, thickness):
    image = cv2.imread(input_path)

    if image is None:
        print("Error: Unable to open image file.")
        return

    cv2.ellipse(image, center, axes, angle, start_angle, end_angle, color, thickness)

    if cv2.imwrite(output_path, image):
        print(f"Image saved to {output_path}")
        cv2.imshow('Image Preview', cv2.imread(output_path))
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    else:
        print("Error: Unable to save image file.")

draw_ellipse(
    '/home/jetson/opencv/images/hemihex_logo.png',
    '/home/jetson/opencv/images/hemihex_logo_ellipse.png',
    (400, 85),
    (375, 75),
    0,
    0,
    360,
    (0, 255, 0),
    2
)
```

------------------------------------------------------------------------

## Summary

-   Ellipses are drawn using `cv2.ellipse()`
-   Supports rotation and partial arcs
-   Useful for rounded annotations and visual overlays

------------------------------------------------------------------------

Maintained by **HemiHex** for OpenCV-based image processing workflows.
