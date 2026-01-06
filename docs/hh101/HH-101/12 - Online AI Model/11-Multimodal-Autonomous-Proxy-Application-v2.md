---
title: Multimodal autonomous proxy application-11
sidebar_position: 11
---

# 7.Multimodal autonomous proxy application

## 1. Concept Introduction

### 1.1 What is an "Autonomous Agent"?

In the `largemodel` project, **multimodal autonomous agents** represent the most advanced form of intelligence. Rather than simply responding to a user's command, they are capable of **autonomously thinking, planning, and sequentially invoking multiple tools to achieve a complex goal**.

The core of this functionality is the `agent_call` tool or its underlying **ToolChainManager`. When a user issues a complex request that cannot be accomplished with a single tool call, the autonomous agent is activated.

### 1.2 Implementation Principles

The autonomous agent implementation in `largemodel` follows the industry-leading **ReAct (Reason + Act)** paradigm. Its core concept is to mimic the human problem-solving process, cycling between "thinking" and "acting."

1. **Reason** : When the agent receives a complex goal, it first "reasons" by invoking a powerful language model (LLM). It asks itself, "What should I do first to achieve this goal? Which tool should I use?" The LLM's output isn't a final answer, but rather an action plan.
2. **Act** : Based on the LLM's reasoning, the agent takes the appropriate action—calling the ToolsManager to run a specified tool (such as visual_positioning).
3. **Observe** : The agent obtains the result of the previous action (the "observation"), for example, ["result": "The cup was found at [120, 300, 180, 360]"].
4. **Rethink** : The agent submits this observation, along with the original goal, to the LLM for a second round of "reflection." It asks itself, "I've found the location of the cup. What should I do next to learn its color?" The LLM might generate a new action plan, such as `{"thought": "I need to analyze the image of the area where the cup is located to determine its color", "action": "seewhat", "args": {"crop_area": [120, 300, 180, 360]}}` .

This **think -> act -> observe** cycle continues until the initial goal is achieved, at which point the agent generates and outputs the final answer.

## 2. Project Architecture

### Key Code

#### 1. Agent Core Workflow ( largemodel/utils/ai_agent.py )

The `_execute_agent_workflow` function is the agent's main execution loop, defining the core "plan -> execute" process.

```python
# From largemodel/utils/ai_agent.py

class AIAgent:
    # ...

    def _execute_agent_workflow(self, task_description: str) -> Dict[str, Any]:
        """
        Executes the agent workflow: Plan -> Execute.
        """
        try:
            # Step 1: Task Planning
            self.node.get_logger().info("AI Agent starting task planning phase")
            plan_result = self._plan_task(task_description)

            # ... (Return early if planning fails)

            self.task_steps = plan_result["steps"]

            # Step 2: Follow all steps in order
            execution_results = []
            tool_outputs = []

            for i, step in enumerate(self.task_steps):
                # 2.1. Processing data references in parameters before execution
                processed_parameters = self._process_step_parameters(step.get("parameters", {}), tool_outputs)
                step["parameters"] = processed_parameters

                # 2.2. Execute a single step
                step_result = self._execute_step(step, tool_outputs)
                execution_results.append(step_result)

                # 2.3. If the step succeeds, save its output for reference in subsequent steps
                if step_result.get("success") and step_result.get("tool_output"):
                    tool_outputs.append(step_result["tool_output"])
                else:
                    # If any step fails, abort the entire task
                    return { "success": False, "message": f"Task terminated because step '{step['description']}' failed." }

            # ... Summarize and return the final result
            summary = self._summarize_execution(task_description, execution_results)
            return { "success": True, "message": summary, "results": execution_results }

        # ... (Exception handling)
```

#### 2. Task Planning and LLM Interaction ( largemodel/utils/ai_agent.py )

The core of the `_plan_task` function is to build a sophisticated prompt, leveraging the large model's inherent reasoning capabilities to generate a structured execution plan.

```python
# From largemodel/utils/ai_agent.py

class AIAgent:
    # ...
    def _plan_task(self, task_description: str) -> Dict[str, Any]:
        """
        Uses the large model for task planning and decomposition.
        """
        # Dynamically generate a list of available tools and their descriptions
        tool_descriptions = []
        for name, adapter in self.tools_manager.tool_chain_manager.tools.items():
            # ... (Get the tool description from adapter.input_schema)
            tool_descriptions.append(f"- {name}({params}): {description}")
        available_tools_str = "\\n".join(tool_descriptions)

        # Build a highly structured plan
        planning_prompt = f"""
As a professional task planning agent, please break down user tasks into a series of specific, executable JSON steps.

**# Available Tools:**
{available_tools_str}

**# Core Rules:**
1. **Data Passing**: When a subsequent step requires the output of a previous step, it must be referenced using the `{{{{steps.N.outputs.KEY}}}}` format.
- `N` is the step ID (starting at 1).
- `KEY` is the specific field name in the output data of the previous step.
2. **JSON Format**: Must strictly return a JSON object.

**# User Tasks:**
{task_description}
"""

        # Calling large models for planning
        messages_to_use = [{"role": "user", "content": planning_prompt}]
        # Note that the general text reasoning interface is called here
        result = self.node.model_client.infer_with_text("", message=messages_to_use)

        # ... (parses the JSON response and returns a list of steps)
```

#### 3. Parameter processing and data flow implementation( largemodel/utils/ai_agent.py )

`_process_step_parameters`The function is responsible for parsing placeholders and realizing data flow between steps.

```python
# From largemodel/utils/ai_agent.py

class AIAgent:
    # ...
    def _process_step_parameters(self, parameters: Dict[str, Any], previous_outputs: List[Any]) -> Dict[str, Any]:
        """
        Parses parameter dictionary, finds and replaces all {{...}} references.
        """
        processed_params = parameters.copy()
        # Regular expression used to match placeholders in the format {{steps.N.outputs.KEY}}
        pattern = re.compile(r"\\{\\{steps\\.(\\d+)\\.outputs\\.(.+?)\\}\\}")

        for key, value in processed_params.items():
            if isinstance(value, str) and pattern.search(value):
                # Use re.sub and a replacement function to process all found placeholders
                # The replacement function will look up and return the value from the previous_outputs list
                processed_params[key] = pattern.sub(replacer_function, value)

        return processed_params
```

### Code Analysis

The AI Agent is the "brain" of the system, translating high-level, sometimes ambiguous, tasks posed by the user into a precise, ordered series of tool calls. Its implementation is independent of any specific model platform and built on a general, extensible architecture.

1. **Dynamic Task Planning**: The Agent's core capability lies in the `_plan_task` function. Rather than relying on hard-coded logic, it dynamically generates task plans by interacting with a larger model.
  - **Self-Awareness and Prompt Construction** : At the beginning of planning, the Agent first examines all available tools and their descriptions. It then packages this tool information, the user's task, and strict rules (such as data transfer format) into a highly structured `planning_prompt` .
  - **Model as Planner** : This prompt is fed into a general text-based model. The model reasoned based on the provided context and returned a multi-step action plan in JSON format. This design is highly scalable: as tools are added or modified in the system, the Agent's planning capabilities are automatically updated without requiring code modifications.
2. **Toolchain and Data Flow**: Real-world tasks often require the collaboration of multiple tools. For example, "take a picture and describe" requires the output (image path) of the "take a picture" tool to be used as the input of the "describe" tool. The AI Agent elegantly implements this through the `_process_step_parameters` function.
  - **Data Reference Placeholders** : During the planning phase, large models embed special placeholders, such as `{{steps.1.outputs.data}}` , in parameter values where data needs to be passed.
  - **Real-Time Parameter Replacement** : In the `_execute_agent_workflow` main loop, `_process_step_parameters` is called before each step. It uses regular expressions to scan all parameters of the current step. Upon discovering a placeholder, it finds the corresponding data from the output list of the previous step and replaces it in real time. This mechanism is key to automating complex tasks.
3. **Supervised Execution and Fault Tolerance**: `_execute_agent_workflow` constitutes the Agent's main execution loop. It strictly follows the planned sequence of steps, executing each action sequentially and ensuring data is correctly passed between them.
  - **Atomic Steps** : Each step is treated as an independent "atomic operation." If any step fails, the entire task chain immediately aborts and reports an error. This ensures system stability and predictability, preventing continued execution in an erroneous state.

In summary, the general implementation of the AI Agent demonstrates an advanced software architecture: rather than solving a problem directly, it builds a framework that enables an external, general-purpose reasoning engine (the large model) to solve the problem. Through two core mechanisms, dynamic programming and data flow management, the Agent orchestrates a series of independent tools into complex workflows capable of completing advanced tasks.

## 3. Practical Practice

### 3.1 Configuring Online LLM

1. **First, obtain an API key from any of the platforms described in the previous tutorials**
2. **Next, you need to update the key in the configuration file. Open the model interface configuration file large_model_interface.yaml**: xxxxxxxxxxvim ~/hemihex_ws/src/largemodel/config/large_model_interface.yaml
3. **Enter your API Key**:
Find the corresponding section and paste the API Key you just copied. This example uses the Tongyi Qianwen configuration. xxxxxxxxxx# large_model_interface.yaml## Thousand Questions on Tongyiqianwen_api_key: "sk-xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx" # Paste your Keyqianwen_model: "qwen-vl-max-latest" # You can choose the model as needed, such as qwen-turbo, qwen-plus
4. **Open the main configuration file HemiHex.yaml**: xxxxxxxxxxvim ~/hemihex_ws/src/largemodel/config/HemiHex.yaml
5. **Select the online platform you want to use**:
Change the `llm_platform` parameter to the platform name you want to use. xxxxxxxxxx# HemiHex.yamlmodel_service:  ros__parameters:    # ...    llm_platform: 'tongyi'  #Optional platforms: 'ollama', 'tongyi', 'spark', 'qianfan', 'openrouter'

### 3.2 Launching and Testing the Functionality

1. **Start the largemodel main program**: Open a terminal and run the following command: xxxxxxxxxxros2 launch largemodel largemodel_control.launch.py
2. **Test**:
  - **Wake up**: Say "Hi,HemiHex" into the microphone.
  - **Dialogue**: After the speaker responds, you can say: `Generate an image similar to the current environment.`
  - **Observe the log**: In the terminal running the `launch` file, you should see the following: The system receives the text command, invokes the `aiagent` tool, and then provides a prompt to the LLM. The LLM analyzes the detailed tool invocation steps. For example, in this question, the `seewhat` tool is invoked to obtain the image, which is then parsed by the LLM. The parsed text is then passed to the LLM as the content of the new image, which is then generated.
  - **Listen to the answer**: After a while, you should hear the completed answer from the speaker, and the camera feed and the newly generated image will pop up. You can later find the newly generated image in the `/home/jetson/hemihex_ws/src/largemodel/resources_file/generated_images` directory.
