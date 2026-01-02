---
sidebar_position: 14
title: Image Binarization with OpenCV
---

# Image Binarization with OpenCV

This section explains how to **convert a grayscale image into a binary
image** using OpenCV.

Image binarization is commonly used for: - Document processing - Edge
and shape detection - Image segmentation - Preprocessing for OCR and
classical vision algorithms

------------------------------------------------------------------------

## 1. Implementation Principle

OpenCV provides the `cv2.threshold()` function to perform image
binarization.

``` python
retval, dst = cv2.threshold(src, thresh, maxval, type)
```

Parameters: - `src`: input grayscale image\
- `thresh`: threshold value\
- `maxval`: value assigned to pixels above the threshold\
- `type`: thresholding method

------------------------------------------------------------------------

## 2. Implementation Effect

Navigate to the OpenCV working directory:

``` bash
cd ~/opencv
```

Run the image binarization script:

``` bash
python3 09.image_binarize.py
```

:::note
Select the image window and press **`q`** to exit the program.
:::

![Image Binarization
Result](/img/docs/jetson/06-OpenCV/6-9/image-20250106161124620.png)

------------------------------------------------------------------------

## 3. Implementation Code

``` python
import cv2

def binarize_image(input_path, output_path, threshold):
    image = cv2.imread(input_path, cv2.IMREAD_GRAYSCALE)

    if image is None:
        print("Error: Unable to open image file.")
        return

    _, binary_image = cv2.threshold(
        image,
        threshold,
        255,
        cv2.THRESH_BINARY
    )

    if cv2.imwrite(output_path, binary_image):
        print(f"Image saved to {output_path}")
        cv2.imshow('Image Preview', cv2.imread(output_path))
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    else:
        print("Error: Unable to save image file.")

binarize_image(
    '/home/jetson/opencv/images/hemihex_logo.png',
    '/home/jetson/opencv/images/hemihex_logo_binarize.png',
    127
)
```

------------------------------------------------------------------------

## 4. Code Explanation

-   `cv2.imread(..., cv2.IMREAD_GRAYSCALE)` loads the image in
    grayscale\
-   `cv2.threshold()` applies binary thresholding\
-   `cv2.imwrite()` saves the output image\
-   Display functions preview the result

------------------------------------------------------------------------

Maintained by **HemiHex** for OpenCV-based image processing workflows.
