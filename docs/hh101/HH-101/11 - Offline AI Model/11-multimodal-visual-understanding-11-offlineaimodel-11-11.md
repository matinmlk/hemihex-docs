---
title: Multimodal Visual Understanding Application
sidebar_position: 11
---

# Multimodal Visual Understanding Application

## 1. Concept Introduction

### 1.1 What is "Visual Understanding"?

In the `largemodel` project, the **multimodal visual understanding** feature enables a robot to go beyond simply processing pixels and instead **understand objects, scenes, and relationships** within an image. This capability allows the system to generate meaningful, natural-language descriptions of what it observes.

The core tool enabling this feature is `seewhat`. When a user issues a command such as **"see what's here"**, this tool is invoked to capture a live image and analyze it using a multimodal AI model.

### 1.2 Implementation Principle Overview

This feature combines **visual information (images)** and **linguistic information (text)** and feeds them into a multimodal large model (for example, LLaVA).

1. **Image Encoding**  
   The model uses a vision encoder to convert the input image into digital feature vectors describing color, shape, and texture.

2. **Text Encoding**  
   The user's question (for example, **"What's on the table?"**) is encoded into a text vector.

3. **Cross-modal Fusion**  
   Image and text vectors are fused using an attention mechanism. The model learns which regions of the image are relevant to the user's question.

4. **Answer Generation**  
   A large language model generates a natural-language description based on the fused visual and textual information.

In short, the system **aligns text with relevant regions of the image and then describes those regions using language**.

---

## 2. Code Analysis

### Key Code

#### 2.1 Tool Layer Entry (`largemodel/utils/tools_manager.py`)

The `seewhat` function defines the execution flow of the visual understanding tool.

```python
# From largemodel/utils/tools_manager.py

class ToolsManager:
    # ...

    def seewhat(self):
        """
        Capture a camera frame and analyze the environment with an AI model.

        :return: A dictionary containing the scene description and image path,
                 or None if the operation fails.
        """
        self.node.get_logger().info("Executing seewhat() tool")
        image_path = self.capture_frame()

        if image_path:
            # Use an isolated context for image analysis.
            analysis_text = self._get_actual_scene_description(image_path)

            # Return structured data for the tool chain.
            return {
                "description": analysis_text,
                "image_path": image_path
            }
        else:
            # Error handling
            return None

    def _get_actual_scene_description(self, image_path, message_context=None):
        """
        Get an AI-generated scene description for the captured image.

        :param image_path: Path to the captured image file.
        :return: Plain-text description of the scene.
        """
        try:
            # Build the prompt (omitted here for brevity)
            result = self.node.model_client.infer_with_image(
                image_path,
                scene_prompt,
                message=simple_context
            )
            # Process the result (omitted)
            return description
        except Exception:
            # Error handling
            pass
```

#### 2.2 Model Interface Layer (`largemodel/utils/large_model_interface.py`)

The `infer_with_image` function is the unified entry point for all image-understanding tasks. It dispatches requests based on the configured model platform.

```python
# From largemodel/utils/large_model_interface.py

class model_interface:
    # ...

    def infer_with_image(self, image_path, text=None, message=None):
        """Unified image inference interface."""
        # Prepare messages (omitted)
        try:
            # Select implementation based on configured platform
            if self.llm_platform == 'ollama':
                response_content = self.ollama_infer(
                    self.messages,
                    image_path=image_path
                )
            elif self.llm_platform == 'tongyi':
                # Logic for the Tongyi platform
                pass

            return {
                "response": response_content,
                "messages": self.messages.copy()
            }
        except Exception:
            # Error handling
            pass
```

### Code Architecture Summary

The implementation follows a **two-layer architecture**:

- **Tool Layer (`tools_manager.py`)**  
  Defines *what* the system does: capture images, prepare prompts, and request analysis.

- **Model Interface Layer (`large_model_interface.py`)**  
  Defines *how* the system communicates with the AI model and selects the appropriate backend.

This separation allows the same business logic to work across **offline and online AI platforms** without code changes.

---

## 3.1 Configuring the Offline Large Model

### 3.1.1 Configuring the LLM Platform (`hemihex.yaml`)

This configuration file determines which large-model platform is used by the model service.

1. **Open the configuration file**:

```bash
vim ~/hemihex_ws/src/largemodel/config/hemihex.yaml
```

2. **Confirm or modify the platform setting**:

```yaml
model_service:
  ros__parameters:
    language: "en"               # Large model interface language
    useolinetts: false           # Not used in text-only mode; can be ignored if not applicable
    llm_platform: "ollama"       # Key: set to "ollama" for offline mode
    regional_setting: "international"
```

### 3.1.2 Configuring the Model Interface (`large_model_interface.yaml`)

This file defines which vision model is used when the `ollama` platform is selected.

1. **Open the file**:

```bash
vim ~/hemihex_ws/src/largemodel/config/large_model_interface.yaml
```

2. **Set or confirm the Ollama vision model** (example):

```yaml
# Offline Large Models (Ollama)
ollama_model: "llava"  # Set to the multimodal model you downloaded (e.g., "llava")
```

:::note
Ensure the configured model supports multimodal (image + text) input.
:::

---

## 3.2 Starting and Testing the Function (Text Input Mode)

:::note
On lower-memory devices (for example, Jetson Orin Nano 4GB), this feature may run slowly or be unstable. For best results, use a higher-performance device or the online model mode if available.
:::

1. **Start the `largemodel` main program (text mode)**:

```bash
ros2 launch largemodel largemodel_control.launch.py text_chat_mode:=true
```

2. **Send a text command** (in a new terminal):

```bash
ros2 run text_chat text_chat
```

Then type:

```text
What do you see
```

3. **Expected behavior**  
In the terminal running the main program, you should see logs indicating the system received the command, invoked the `seewhat` tool, and printed a text description generated by the vision model.

---

## 4. Common Problems and Solutions

### Problem 1: The log displays "Failed to call ollama vision model" or the connection is refused

**Possible causes**
- Ollama service is not running.
- The configured model is missing or misspelled.
- Port binding or local firewall restrictions.

**Solution**
1. Confirm the model exists and Ollama is available:

```bash
ollama list
```

2. Re-check configuration values:
- `hemihex.yaml`: `llm_platform: "ollama"`
- `large_model_interface.yaml`: `ollama_model: "llava"` (or your chosen model)

### Problem 2: The `seewhat` tool returns "Unable to open camera" or fails to capture

**Possible causes**
- Camera not detected.
- Camera is busy (used by another app).
- Permission or device access issue.

**Solution**
1. Verify the camera device exists:

```bash
ls /dev/video*
```

2. Test the camera with a viewer app (for example, `cheese` or `guvcview`) and close any applications using the camera before retrying.

---

This documentation is maintained by **HemiHex** and describes a modular, platform-agnostic approach to multimodal visual understanding on Jetson-based systems.
