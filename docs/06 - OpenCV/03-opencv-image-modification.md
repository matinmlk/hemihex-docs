---
sidebar_position: 8
title: 03 - Image Modification with OpenCV
---

# Image Modification with OpenCV

This section explains how to **modify image pixel values** using OpenCV
and NumPy slicing in Python.

Image modification is a fundamental operation in computer vision and is
commonly used for:

-   Region masking\
-   Drawing overlays\
-   Data preprocessing\
-   Debug visualization

------------------------------------------------------------------------

## 1. Implementation Principle

Images loaded by OpenCV are stored as **NumPy arrays**.

This means you can directly:

-   Access pixels by index\
-   Slice regions of interest (ROI)\
-   Assign new pixel values

By modifying array values, the image content changes immediately.

------------------------------------------------------------------------

## 2. Implementation Effect

Navigate to the OpenCV working directory:

``` bash
cd ~/opencv
```

Run the image modification script:

``` bash
python3 03.image_modify.py
```

::: note
Select the image display window and press **`q`** to exit the program.
:::

![Image Modification Result](/img/docs/jetson/06-OpenCV/6-3/image-20250106144557431.png)

------------------------------------------------------------------------

## 3. Implementation Code

``` python
import cv2

def modify_image(input_path, output_path):
    image = cv2.imread(input_path)

    if image is None:
        print("Error: Unable to open image file.")
        return

    # Modify the top-left 50x50 region (set to white)
    image[:50, :50] = [255, 255, 255]

    if cv2.imwrite(output_path, image):
        print(f"Image saved to {output_path}")
        cv2.imshow('Image Preview', cv2.imread(output_path))
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    else:
        print("Error: Unable to save image file.")

modify_image(
    '/home/jetson/opencv/images/hemihex_logo.png',
    '/home/jetson/opencv/images/hemihex_logo_modify.png'
)
```

------------------------------------------------------------------------

## 4. Code Explanation

-   `cv2.imread()` loads the image into memory as a NumPy array\
-   Array slicing selects the region of interest\
-   `cv2.imwrite()` saves the modified image\
-   Display functions preview the result

------------------------------------------------------------------------

Maintained by **HemiHex** for OpenCV-based image processing workflows.
