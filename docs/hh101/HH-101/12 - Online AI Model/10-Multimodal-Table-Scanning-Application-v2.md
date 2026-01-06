---
title: Multimodal table scanning application-10
sidebar_position: 10
---

# 6.Multimodal table scanning application

## 1. Concept Introduction

### 1.1 What is "Multimodal Table Scanning"?

**Multimodal table scanning** is a technology that uses image processing and artificial intelligence to identify and extract table information from images or PDF documents. It not only focuses on visual table structure recognition but also integrates multimodal data such as text content and layout information to enhance table understanding. **Large Language Models (LLMs)** provide powerful semantic analysis capabilities to understand this extracted information. The two complement each other and enhance the intelligent level of document processing.

### 1.2 Implementation Principles

1. **Table Detection and Content Recognition**
  - Utilizes computer vision technology to locate tables in documents and uses optical character recognition (OCR) technology to convert the text within the tables into an editable format.
  - Utilizes deep learning methods to analyze the table structure (row and column division, cell merging, etc.) and generate a structured data representation.
2. **Multimodal Fusion**
  - Integrate visual information (such as table layout), text (OCR results), and any metadata (such as file type and source) to form a comprehensive view of the data.
  - Use a specially designed multimodal model (such as LayoutLM) to simultaneously process these different types of data to more accurately understand the table content and its context.

## 2. Code Analysis

### Key Code

#### 1. Tools Layer Entry ( largemodel/utils/tools_manager.py )

The `scan_table` function in this file defines the tool's execution flow, specifically how it constructs a prompt that returns a Markdown-formatted result.

```python
# From largemodel/utils/tools_manager.py
class ToolsManager:
    # ...
    def scan_table(self, args):
        """
        Scan a table from an image and save the content as a Markdown file.

        :param args: Arguments containing the image path.
        :return: Dictionary with file path and content.
        """
        self.node.get_logger().info(f"Executing scan_table() tool with args: {args}")
        try:
            image_path = args.get("image_path")
            # ... (Path checking and fallback)

            # Construct a prompt asking the large model to recognize the table and return it in Markdown format.
            if self.node.language == 'zh':
                prompt = "Please carefully analyze this image, identify the table within it, and return its content in Markdown format.  # translated from Chinese"
            else:
                prompt = "Please carefully analyze this image, identify the table within it, and return its content in Markdown format."

            result = self.node.model_client.infer_with_image(image_path, prompt)

            # ... (Extract Markdown text from the results)

            # Save the recognized content to a Markdown file.
            md_file_path = os.path.join(self.node.pkg_path, "resources_file", "scanned_tables", f"table_{timestamp}.md")
            with open(md_file_path, 'w', encoding='utf-8') as f:
                f.write(table_content)

            return {
                "file_path": md_file_path,
                "table_content": table_content
            }
        # ... (Error Handling)
```

#### 2. Model interface layer ( largemodel/utils/large_model_interface.py )

The `infer_with_image` function in this file is the unified entry point for all image-related tasks.

```python
# From largemodel/utils/large_model_interface.py

class model_interface:
    # ...
    def infer_with_image(self, image_path, text=None, message=None):
        """Unified image inference interface."""
        # ... (Prepare message)
        try:
            # Determine which specific implementation to call based on the value of self.llm_platform
            if self.llm_platform == 'ollama':
                response_content = self.ollama_infer(self.messages, image_path=image_path)
            elif self.llm_platform == 'tongyi':
                # ... Logic for calling the Tongyi model
                pass
            # ... (Logic for other platforms)
        # ...
        return {'response': response_content, 'messages': self.messages.copy()}
```

### Code Analysis

The table scanning function is a typical application for converting unstructured image data into structured text data. Its core technology remains **guiding model behavior through prompt engineering**.

1. **Tools Layer (tools_manager.py)**:
  - The `scan_table` function is the business process controller for this function. It receives an image containing a table as input.
  - The key operation of this function is **building a targeted prompt** . This prompt directly instructs the large model to perform two tasks: 1. Recognize the table in the image. 2. Return the recognized content in Markdown format. This mandatory output format is key to achieving unstructured-to-structured conversion.
  - After constructing the prompt, it calls the `infer_with_image` method of the model interface layer, passing the image and the formatting instructions.
  - After receiving the Markdown text returned from the model interface layer, it performs a file operation: writing the text content to a new `.md` file.
  - Finally, it returns structured data containing the new file path and table contents.
2. **Model Interface Layer (large_model_interface.py)**:
  - The `infer_with_image` function continues to serve as the unified "dispatching center." It receives the image and prompt from `scan_table` and dispatches the task to the correct backend model implementation based on the current system configuration ( `self.llm_platform` ).
  - Regardless of the backend model, this layer handles the communication details with the specific platform, ensuring that the image and text data are sent correctly, and then returns the plain text (in this case, Markdown-formatted text) returned by the model to the tooling layer.

In summary, the general workflow for table scanning is: ToolsManager receives an image and constructs a command to convert the table in this image to Markdown. ToolsManager calls the model interface. ModelInterface packages the image and the command and sends them to the corresponding model platform according to the configuration. The model returns Markdown-formatted text. ModelInterface returns the text to ToolsManager. ToolsManager saves the text as a .md file and returns the result. This workflow demonstrates how to leverage the formatting capabilities of the large model as a powerful OCR (Optical Character Recognition) and data structuring tool.

## 3. Practical Application

### 3.1 Configuring Online LLM

1. **First, obtain the API key from any of the platforms described in the previous tutorials.**
2. **Next, you need to update the key in the configuration file. Open the model interface configuration file large_model_interface.yaml**: xxxxxxxxxxvim ~/hemihex_ws/src/largemodel/config/large_model_interface.yaml
3. **Enter your API Key**:
Find the corresponding section and paste the API Key you just copied. This example uses the Tongyi Qianwen configuration. xxxxxxxxxx# large_model_interface.yaml## Thousand Questions on Tongyiqianwen_api_key: "sk-xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx" # Paste your Keyqianwen_model: "qwen-vl-max-latest" # You can choose the model as needed, such asqwen-turbo, qwen-plus
4. **Open the main configuration file HemiHex.yaml**: xxxxxxxxxxvim ~/hemihex_ws/src/largemodel/config/HemiHex.yaml
5. **Select the online platform you want to use**:
Change the `llm_platform` parameter to the platform name you want to use. xxxxxxxxxx# HemiHex.yamlmodel_service:  ros__parameters:    # ...    llm_platform: 'tongyi'  #Optional platforms:'ollama', 'tongyi', 'spark', 'qianfan', 'openrouter'

### 3.2 Launch and test the function

1. **Prepare the image file**: Place the image file to be tested in the following directory:`/home/jetson/hemihex_ws/src/largemodel/resources_file/scan_table` Then name the image `test_table.jpg`
2. **Start the largemodel main program**: Open a terminal and run the following command: xxxxxxxxxxros2 launch largemodel largemodel_control.launch.py
3. **Test**:
  - **Wake up**: Say "Hi,HemiHex" into the microphone.
  - **Dialogue**: After the speaker responds, you can say, "Analyze the table."
  - **Observe the log**: In the terminal running the `launch` file, you should see the following:
    1. The ASR node recognizes your question and prints it.
    2. The `model_service` node receives the text, calls the LLM, and prints the LLM's response.
  - **Listen for the answer**: After a while, you should hear the answer from the speaker and find an md file containing the table information in the `/home/jetson/hemihex_ws/src/largemodel/resources_file/scan_table` path.
