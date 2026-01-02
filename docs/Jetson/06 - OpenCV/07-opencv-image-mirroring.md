---
sidebar_position: 12
title: Image Mirroring with OpenCV
---

# Image Mirroring with OpenCV

This section explains how to **mirror (flip) an image** using OpenCV.

Image mirroring is commonly used for: - Data augmentation - Symmetry
analysis - Preprocessing for vision algorithms

------------------------------------------------------------------------

## 1. Implementation Principle

OpenCV provides the `cv2.flip()` function to flip an image.

``` python
cv2.flip(src, flipCode)
```

Where:

-   `src` is the input image
-   `flipCode` determines the flip direction:
    -   `0` → vertical flip\
    -   `1` → horizontal flip\
    -   `-1` → both vertical and horizontal

------------------------------------------------------------------------

## 2. Implementation Effect

Navigate to the OpenCV working directory:

``` bash
cd ~/opencv
```

Run the image mirroring script:

``` bash
python3 07.image_flip.py
```

:::note
Select the image window and press **`q`** to exit the program.
:::

![Image Mirroring
Result](/img/docs/jetson/06-OpenCV/6-7/image-20250106155337268.png)

------------------------------------------------------------------------

## 3. Implementation Code

``` python
import cv2

def flip_image(input_path, output_path, flip_code):
    image = cv2.imread(input_path)

    if image is None:
        print("Error: Unable to open image file.")
        return

    flipped_image = cv2.flip(image, flip_code)

    if cv2.imwrite(output_path, flipped_image):
        print(f"Image saved to {output_path}")
        cv2.imshow('Image Preview', cv2.imread(output_path))
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    else:
        print("Error: Unable to save image file.")

flip_image(
    '/home/jetson/opencv/images/hemihex_logo.png',
    '/home/jetson/opencv/images/hemihex_logo_flip.png',
    1
)
```

------------------------------------------------------------------------

## 4. Code Explanation

-   `cv2.imread()` loads the source image\
-   `cv2.flip()` mirrors the image based on the flip code\
-   `cv2.imwrite()` saves the flipped image\
-   Display functions preview the result

------------------------------------------------------------------------

## Summary

-   Image mirroring flips an image horizontally or vertically
-   Implemented using `cv2.flip()`
-   Useful for augmentation and preprocessing
-   Flip direction is controlled by `flipCode`

------------------------------------------------------------------------

Maintained by **HemiHex** for OpenCV-based image processing workflows.
