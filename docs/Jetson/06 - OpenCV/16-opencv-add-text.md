---
sidebar_position: 20
title: Add Text with OpenCV
---

# Add Text with OpenCV

This section explains how to **draw text on an image** using OpenCV in
Python.

Adding text is commonly used for: - Labeling images - Displaying status
or metadata - Debugging and visualization in computer vision pipelines

------------------------------------------------------------------------

## 1. Implementation Principle

OpenCV provides the `cv2.putText()` function to draw text directly on an
image.

``` python
cv2.putText(image, text, position, font, font_scale, color, thickness, line_type)
```

Where: - `image` is the input image - `text` is the string to be drawn -
`position` is the bottom-left corner of the text `(x, y)` - `font`
specifies the font type (e.g. `cv2.FONT_HERSHEY_SIMPLEX`) - `font_scale`
controls text size - `color` is the text color in **BGR** format -
`thickness` defines line thickness - `line_type` controls line
smoothness (e.g. `cv2.LINE_AA`)

------------------------------------------------------------------------

## 2. Implementation Effect

Navigate to the OpenCV working directory:

``` bash
cd ~/opencv
```

Run the text drawing script:

``` bash
python3 16.image_draw_text.py
```

::: note
Select the image window and press **`q`** to exit the program.
:::

![Add Text
Result](/img/docs/jetson/06-OpenCV/6-16/image-20250106184848293.png)

------------------------------------------------------------------------

## 3. Implementation Code

``` python
import cv2

def draw_text(input_path, output_path, text, position, font, font_scale, color, thickness):
    image = cv2.imread(input_path)

    if image is None:
        print("Error: Unable to open image file.")
        return

    cv2.putText(
        image,
        text,
        position,
        font,
        font_scale,
        color,
        thickness,
        cv2.LINE_AA
    )

    if cv2.imwrite(output_path, image):
        print(f"Image saved to {output_path}")
        cv2.imshow('Image Preview', cv2.imread(output_path))
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    else:
        print("Error: Unable to save image file.")

draw_text(
    '/home/jetson/opencv/images/hemihex_logo.png',
    '/home/jetson/opencv/images/hemihex_logo_text.png',
    'Hello, OpenCV!',
    (550, 150),
    cv2.FONT_HERSHEY_SIMPLEX,
    1,
    (0, 255, 0),
    2
)
```

------------------------------------------------------------------------

## 4. Code Explanation

-   `cv2.imread()` loads the source image\
-   `cv2.putText()` draws the text\
-   `cv2.imwrite()` saves the annotated image\
-   Display functions preview the result

------------------------------------------------------------------------

Maintained by **HemiHex** for OpenCV-based image processing workflows.
