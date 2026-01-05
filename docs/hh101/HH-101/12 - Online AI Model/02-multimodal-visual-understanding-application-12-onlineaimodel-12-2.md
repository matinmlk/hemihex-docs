---
title: Multimodal Video Understanding
sidebar_position: 0
---

# 2.Multimodal visual understand application

## 1. Concept Introduction

### 1.1 What is "Video Analysis"?

In the`largemodel`project, the**multimodal video analysis**feature enables a robot to process a video and summarize its core content, describe key events, or answer specific questions about the video in natural language. This allows the robot to leap from understanding only static images to understanding dynamic and temporal relationships.

The core tool for this feature is`analyze_video`. When a user provides a video file and asks a question (such as "Summarize what this video says"), the system invokes this tool to process and analyze the video and return a textual response from the AI.

### 1.2 Implementation Principles

The core challenge of offline video analysis lies in how to efficiently process video data containing hundreds or thousands of frames. A popular implementation principle is as follows:

1. **Keyframe Extraction**
: First, the system does not process every frame of the video. Instead, it extracts the most representative keyframes through algorithms (such as scene boundary detection or fixed-time sampling). This significantly reduces the amount of data required for processing.
2. **Image Encoding**
: Each extracted keyframe is fed into a visual encoder, similar to visual understanding applications, and converted into a digital vector containing image information.
3. **Temporal Information Fusion**
: This is the most significant difference from single-image understanding. The model needs to understand the temporal order between these keyframes. Typically, a recurrent neural network (RNN) or Transformer model is used to fuse the vectors of all keyframes to form a "memory" vector that represents the dynamic content of the entire video.
4. **Question-Answering and Generation**
: The user's text question is encoded and then cross-modally fused with this "memory" vector. Finally, the language model generates a summary of the entire video or an answer to a specific question based on this fused information.

Simply put, it is to**condense a video into several key pictures and their sequence, and then understand the whole story like reading a comic strip and answer related questions**.

---

## 2. Code Analysis

### Key Code

#### 1. Tool Layer Entry ( largemodel/utils/tools_manager.py )

The`analyze_video`function in this file defines the execution flow of the tool.

```python
# From largemodel/utils/tools_manager.py
class ToolsManager:
    # ...
    def analyze_video(self, args):
        """
        Analyze video file and provide content description.
        Analyzes video files and provides a description of the content.

        :param args: Arguments containing video path.
        :return: Dictionary with video description and path.
        """
        self.node.get_logger().info(f"Executing analyze_video() tool with args: {args}")
        try:
            video_path = args.get("video_path")
            # ... (Intelligent path fallback mechanism)

            if video_path and os.path.exists(video_path):
                # ... (Build Prompt)

                # Use a fully isolated, one-time context for video analysis to ensure a plain text description.
                simple_context = [{
                    "role": "system",
                    "content": "You are a video description assistant. ..."
                }]

                result = self.node.model_client.infer_with_video(video_path, prompt, message=simple_context)

                # ... (processing results)
                return {
                    "description": description,
                    "video_path": video_path
                }
            # ... (error handling)
```

#### 2. Model interface layer and frame extraction ( largemodel/utils/large_model_interface.py )

The functions in this file are responsible for processing the video files and passing them to the underlying model.

```python
# From largemodel/utils/large_model_interface.py

class model_interface:
    # ...
    def infer_with_video(self, video_path, text=None, message=None):
        """Unified video inference interface. """
        # ... (prepare message)
        try:
           # Determine which specific implementation to call based on self.llm_platform
            if self.llm_platform == 'ollama':
                response_content = self.ollama_infer(self.messages, video_path=video_path)
            # ... (Logic for other online platforms)
        # ...
        return {'response': response_content, 'messages': self.messages.copy()}

    def _extract_video_frames(self, video_path, max_frames=5):
        """Extract keyframes from a video for analysis. """
        try:
            import cv2
            # ... (video reading and frame interval calculation)
            while extracted_count < max_frames:
                # ... (Loop reading video frames)
                if frame_count % frame_interval == 0:
                    # ... (save the frame as a temporary image)
                    frame_base64 = self.encode_file_to_base64(temp_path)
                    frame_images.append(frame_base64)
            # ...
            return frame_images
        # ... (Exception handling)
```

### Code Analysis

The implementation of video analysis is more complex than image analysis. It requires a key preprocessing step at the model interface layer: frame extraction.

1. **Tools Layer (tools_manager.py)**:
  - The
`analyze_video`
function is the entry point for video analysis. Its responsibilities are clear: accept a video file path and construct a prompt to request a description of the video content.
  - It initiates the analysis process by calling the
`self.node.model_client.infer_with_video`
method. Like the visual understanding tool, it is completely agnostic about the underlying model details and only handles passing the "video file" and "analysis instructions."
2. **Model Interface Layer (large_model_interface.py)**:
  - The
`infer_with_video`
function is the scheduler that connects the upper-level tools with the underlying model. It dispatches tasks to the corresponding implementation functions based on the platform configuration (
`self.llm_platform`
).
  - Unlike processing single images, processing videos requires an additional step. The
`_extract_video_frames`
method demonstrates the general logic for implementing this step: it uses the
`cv2`
library to read the video and extract several keyframes (by default, five).
  - Each extracted frame is treated as a separate image and typically encoded as a Base64 string.
  - Ultimately, a request containing multiple frame image data and analysis instructions is sent to the main model. The model performs a comprehensive analysis of these consecutive images to generate a description of the entire video content.
  - This "video-to-multiple-images" preprocessing is completely encapsulated in the model interface layer and is transparent to the tool layer.

In summary, the general process of video analysis is:`ToolsManager`initiates an analysis request ->`model_interface`intercepts the request and calls`_extract_video_frames`to decompose the video file into multiple keyframe images ->`model_interface`sends these images, along with analysis instructions, to the corresponding model platform according to the configuration -> the model returns a comprehensive description of the video -> the results are finally returned to`ToolsManager`. This design ensures the stability and versatility of upper-layer applications.

## 3. Practical Operations

### 3.1 Configuring Online LLM

1. **First, obtain an API key from any platform described in the previous tutorial**
2. **Then, you need to update the key in the configuration file and open the model interface configuration file.large_model_interface.yaml**:
xxxxxxxxxxvim~/yahboom_ws/src/largemodel/config/large_model_interface.yaml
3. **Enter your API Key**:Find the corresponding section and paste the API Key you just copied. This example uses the Tongyi Qianwen configuration.
x# large_model_interface.yaml## Thousand Questions on Tongyiqianwen_api_key:"sk-xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx"# Paste your Keyqianwen_model:"qwen-vl-max-latest"# You can choose the model as needed, such as qwen-turbo, qwen-plus
4. **Open the main configuration file yahboom.yaml**:
xxxxxxxxxxvim~/yahboom_ws/src/largemodel/config/yahboom.yaml
5. **Select the online platform you want to use**:Change the`llm_platform`parameter to the platform name you want to use.
xxxxxxxxxx# yahboom.yamlmodel_service:ros__parameters:# ...llm_platform:'tongyi'#Optional platforms: 'ollama', 'tongyi', 'spark', 'qianfan', 'openrouter'

### 3.2 Launching and Testing the Functionality

1. **Launch the largemodel main program and enable text interaction mode**:
xxxxxxxxxxros2 launch largemodel largemodel_control.launch.py text_chat_mode:=true
2. **Send text command**:Open another terminal and run the following command:
xxxxxxxxxxros2 run text_chat text_chat

Then, start typing your question.

3. **Test**:
  - Type your question in the terminal and press Enter. For example:
`Analyze this video`
  - Watch the terminal output. After a few moments, you should see a detailed answer returned from the cloud-based model.
