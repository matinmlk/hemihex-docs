---
title: Multimodal Autonomous Proxy Application
sidebar_position: 0
---

# 9.Multimodal autonomous proxy application

## 1. Concept Introduction

### 1.1 What is an "Autonomous Agent"?

In the largemodel project, multimodal autonomous agents represent the most advanced form of intelligence. Rather than simply responding to a user's command, they are capable of autonomously thinking, planning, and continuously invoking multiple tools to achieve a complex goal .

The core of this functionality is the **agent_call ** tool or its underlying **ToolChainManager . When a user issues a complex request that cannot be accomplished with a single tool call, the autonomous agent is activated.

### 1.2 Implementation Principles

The autonomous agent implementation in largemodel follows the industry-leading ReAct (Reason + Act) paradigm. Its core concept is to mimic the human problem-solving process, cycling between "thinking" and "acting."

This think -> act -> observe cycle continues until the initial goal is achieved, at which point the agent generates and outputs the final answer.

## 2. Code Analysis

### Key Code

#### 1. Agent Core Workflow ( largemodel/utils/ai_agent.py )

The _execute_agent_workflow function is the agent's main execution loop, defining the core "plan -> execute" process.

```bash
# From largemodel/utils/ai_agent.py
​
class
AIAgent
:
# ...
​
def
_execute_agent_workflow
(
self
,
task_description
:
str
)
->
Dict
[
str
,
Any
]:
"""
Executes the agent workflow: Plan -> Execute. / 执行Agent工作流：规划 -> 执行。
"""
try
:
# Step 1: Mission Planning
self
.
node
.
get_logger
().
info
(
"AI Agent starting task planning phase"
)
plan_result
=
self
.
_plan_task
(
task_description
)
# ... (Return early if planning fails)
​
self
.
task_steps
=
plan_result
[
"steps"
]
​
# Step 2: Follow all steps in order
execution_results
= []
tool_outputs
= []
​
for
i
,
step
in
enumerate
(
self
.
task_steps
):
# 2.1. Processing data references in parameters before execution
processed_parameters
=
self
.
_process_step_parameters
(
step
.
get
(
"parameters"
, {}),
tool_outputs
)
step
[
"parameters"
] =
processed_parameters
​
# 2.2. Execute a single step
step_result
=
self
.
_execute_step
(
step
,
tool_outputs
)
execution_results
.
append
(
step_result
)
​
# 2.3. If the step succeeds, save its output for reference in subsequent steps
if
step_result
.
get
(
"success"
)
and
step_result
.
get
(
"tool_output"
):
tool_outputs
.
append
(
step_result
[
"tool_output"
])
else
:
# If any step fails, abort the entire task
return
{
"success"
:
False
,
"message"
:
f"Task terminated because step '{step['description']}' failed."
}
# ... Summarize and return the final result
summary
=
self
.
_summarize_execution
(
task_description
,
execution_results
)
return
{
"success"
:
True
,
"message"
:
summary
,
"results"
:
execution_results
}
​
# ... (Exception handling)
```


```bash
# From largemodel/utils/ai_agent.py
```

```bash
​
```

```bash
class
AIAgent
:
```

```bash
# ...
```

```bash
​
```

```bash
def
_execute_agent_workflow
(
self
,
task_description
:
str
)
->
Dict
[
str
,
Any
]:
```

```bash
"""
```

```bash
Executes the agent workflow: Plan -> Execute. / 执行Agent工作流：规划 -> 执行。
```

```bash
"""
```

```bash
try
:
```

```bash
# Step 1: Mission Planning
```

```bash
self
.
node
.
get_logger
().
info
(
"AI Agent starting task planning phase"
)
```

```bash
plan_result
=
self
.
_plan_task
(
task_description
)
```

```bash
# ... (Return early if planning fails)
```

```bash
​
```

```bash
self
.
task_steps
=
plan_result
[
"steps"
]
```

```bash
​
```

```bash
# Step 2: Follow all steps in order
```

```bash
execution_results
= []
```

```bash
tool_outputs
= []
```

```bash
​
```

```bash
for
i
,
step
in
enumerate
(
self
.
task_steps
):
```

```bash
# 2.1. Processing data references in parameters before execution
```

```bash
processed_parameters
=
self
.
_process_step_parameters
(
step
.
get
(
"parameters"
, {}),
tool_outputs
)
```

```bash
step
[
"parameters"
] =
processed_parameters
```

```bash
​
```

```bash
# 2.2. Execute a single step
```

```bash
step_result
=
self
.
_execute_step
(
step
,
tool_outputs
)
```

```bash
execution_results
.
append
(
step_result
)
```

```bash
​
```

```bash
# 2.3. If the step succeeds, save its output for reference in subsequent steps
```

```bash
if
step_result
.
get
(
"success"
)
and
step_result
.
get
(
"tool_output"
):
```

```bash
tool_outputs
.
append
(
step_result
[
"tool_output"
])
```

```bash
else
:
```

```bash
# If any step fails, abort the entire task
```

```bash
return
{
"success"
:
False
,
"message"
:
f"Task terminated because step '{step['description']}' failed."
}
```

```bash
# ... Summarize and return the final result
```

```bash
summary
=
self
.
_summarize_execution
(
task_description
,
execution_results
)
```

```bash
return
{
"success"
:
True
,
"message"
:
summary
,
"results"
:
execution_results
}
```

```bash
​
```

```bash
# ... (Exception handling)
```

#### 2. Interacting with the LLM in Task Planning ( largemodel/utils/ai_agent.py )

The core of the _plan_task function is to build a sophisticated prompt, leveraging the large model's inherent reasoning capabilities to generate a structured execution plan.

```bash
# From largemodel/utils/ai_agent.py
​
class
AIAgent
:
# ...
def
_plan_task
(
self
,
task_description
:
str
)
->
Dict
[
str
,
Any
]:
"""
Uses the large model for task planning and decomposition. / 使用大模型进行任务规划和分解。
"""
# Dynamically generate a list of available tools and their descriptions
tool_descriptions
= []
for
name
,
adapter
in
self
.
tools_manager
.
tool_chain_manager
.
tools
.
items
():
# ... (Get tool description from adapter.input_schema)
tool_descriptions
.
append
(
f"- {name}({params}): {description}"
)
available_tools_str
=
"\\n"
.
join
(
tool_descriptions
)
​
# Build a highly structured plan
planning_prompt
=
f"""
作为一个专业的任务规划Agent，请将用户任务分解为一系列具体的、可执行的JSON步骤。
​
**# Available Tools:**
{available_tools_str}
​
**# Core Rules:**
1.  **Data Passing**: When a subsequent step needs to use the output of a previous step, you **must** use the `{{{{steps.N.outputs.KEY}}}}` format for referencing.
- `N` is the step ID (starting from 1).
- `KEY` is the specific field name in the output data of the previous step.
- `outputs` can be followed by `data` (for primary data) or `metadata.sub_key` (for metadata).
2.  **JSON Format**: You must strictly return a JSON object. Do not include any Markdown wrappers (like ```json```).
3.  **Tool Selection**: Strictly select the most appropriate tool based on its description.
​
**# User Task:**
{task_description}
"""
# Calling large models for planning
messages_to_use
= [{
"role"
:
"user"
,
"content"
:
planning_prompt
}]
# Note that the general text reasoning interface is called here
result
=
self
.
node
.
model_client
.
infer_with_text
(
""
,
message
=
messages_to_use
)
# ... (Parse the JSON response and return a list of steps)
```


```bash
# From largemodel/utils/ai_agent.py
```

```bash
​
```

```bash
class
AIAgent
:
```

```bash
# ...
```

```bash
def
_plan_task
(
self
,
task_description
:
str
)
->
Dict
[
str
,
Any
]:
```

```bash
"""
```

```bash
Uses the large model for task planning and decomposition. / 使用大模型进行任务规划和分解。
```

```bash
"""
```

```bash
# Dynamically generate a list of available tools and their descriptions
```

```bash
tool_descriptions
= []
```

```bash
for
name
,
adapter
in
self
.
tools_manager
.
tool_chain_manager
.
tools
.
items
():
```

```bash
# ... (Get tool description from adapter.input_schema)
```

```bash
tool_descriptions
.
append
(
f"- {name}({params}): {description}"
)
```

```bash
available_tools_str
=
"\\n"
.
join
(
tool_descriptions
)
```

```bash
​
```

```bash
# Build a highly structured plan
```

```bash
planning_prompt
=
f"""
```

```bash
作为一个专业的任务规划Agent，请将用户任务分解为一系列具体的、可执行的JSON步骤。
```

```bash
​
```

```bash
**# Available Tools:**
```

```bash
{available_tools_str}
```

```bash
​
```

```bash
**# Core Rules:**
```

```bash
1.  **Data Passing**: When a subsequent step needs to use the output of a previous step, you **must** use the `{{{{steps.N.outputs.KEY}}}}` format for referencing.
```

```bash
- `N` is the step ID (starting from 1).
```

```bash
- `KEY` is the specific field name in the output data of the previous step.
```

```bash
- `outputs` can be followed by `data` (for primary data) or `metadata.sub_key` (for metadata).
```

```bash
2.  **JSON Format**: You must strictly return a JSON object. Do not include any Markdown wrappers (like ```json```).
```

```bash
3.  **Tool Selection**: Strictly select the most appropriate tool based on its description.
```

```bash
​
```

```bash
**# User Task:**
```

```bash
{task_description}
```

```bash
"""
```

```bash
# Calling large models for planning
```

```bash
messages_to_use
= [{
"role"
:
"user"
,
"content"
:
planning_prompt
}]
```

```bash
# Note that the general text reasoning interface is called here
```

```bash
result
=
self
.
node
.
model_client
.
infer_with_text
(
""
,
message
=
messages_to_use
)
```

```bash
# ... (Parse the JSON response and return a list of steps)
```

#### 3. Parameter processing and data flow implementation( largemodel/utils/ai_agent.py )

The _process_step_parameters function is responsible for parsing placeholders and implementing data flow between steps.

```bash
# From largemodel/utils/ai_agent.py
​
class
AIAgent
:
# ...
def
_process_step_parameters
(
self
,
parameters
:
Dict
[
str
,
Any
],
previous_outputs
:
List
[
Any
])
->
Dict
[
str
,
Any
]:
"""
Parses parameter dictionary, finds and replaces all {{...}} references.
"""
processed_params
=
parameters
.
copy
()
# Regular expression used to match placeholders in the format {{steps.N.outputs.KEY}}
pattern
=
re
.
compile
(
r"\\{\\{steps\\.(\\d+)\\.outputs\\.(.+?)\\}\\}"
)
​
for
key
,
value
in
processed_params
.
items
():
if
isinstance
(
value
,
str
)
and
pattern
.
search
(
value
):
# Use re.sub and a replacement function to process all found placeholders
#The replacement function looks up and returns a value from the previous_outputs list.
processed_params
[
key
] =
pattern
.
sub
(
replacer_function
,
value
)
return
processed_params
```


```bash
# From largemodel/utils/ai_agent.py
```

```bash
​
```

```bash
class
AIAgent
:
```

```bash
# ...
```

```bash
def
_process_step_parameters
(
self
,
parameters
:
Dict
[
str
,
Any
],
previous_outputs
:
List
[
Any
])
->
Dict
[
str
,
Any
]:
```

```bash
"""
```

```bash
Parses parameter dictionary, finds and replaces all {{...}} references.
```

```bash
"""
```

```bash
processed_params
=
parameters
.
copy
()
```

```bash
# Regular expression used to match placeholders in the format {{steps.N.outputs.KEY}}
```

```bash
pattern
=
re
.
compile
(
r"\\{\\{steps\\.(\\d+)\\.outputs\\.(.+?)\\}\\}"
)
```

```bash
​
```

```bash
for
key
,
value
in
processed_params
.
items
():
```

```bash
if
isinstance
(
value
,
str
)
and
pattern
.
search
(
value
):
```

```bash
# Use re.sub and a replacement function to process all found placeholders
```

```bash
#The replacement function looks up and returns a value from the previous_outputs list.
```

```bash
processed_params
[
key
] =
pattern
.
sub
(
replacer_function
,
value
)
```

```bash
return
processed_params
```

### Code Analysis

The AI ​​Agent is the "brain" of the system, translating high-level, sometimes ambiguous, tasks posed by the user into a precise, ordered series of tool calls. Its implementation is independent of any specific model platform and built on a general, extensible architecture.

Dynamic Task Planning : The Agent's core capability lies in the _plan_task function. Rather than relying on hard-coded logic, it dynamically generates task plans by interacting with a larger model.

Toolchain and Data Flow : Real-world tasks often require the collaboration of multiple tools. For example, "take a picture and describe" requires the output (image path) of the "take a picture" tool to be used as the input of the "describe" tool. The AI ​​Agent elegantly implements this through the _process_step_parameters function.

Supervised Execution and Fault Tolerance : _execute_agent_workflow constitutes the Agent's main execution loop. It strictly follows the planned sequence of steps, executing each action sequentially and ensuring data is correctly passed between them.

In summary, the general implementation of the AI ​​Agent demonstrates an advanced software architecture: rather than solving a problem directly, it builds a framework that enables an external, general-purpose reasoning engine (a large model) to solve the problem. Through two core mechanisms, dynamic programming and data flow management, the Agent orchestrates a series of independent tools into complex workflows capable of completing advanced tasks.

## 3. Practical Operations

### 3.1 Configuring the Offline Large Model

#### 3.1.1 Configuring the LLM Platform ( HemiHex.yaml )

This file determines which large model platform the model_service node loads as its primary language model.

Open the file in the terminal :

```bash
vim
~/yahboom_ws/src/largemodel/config/HemiHex.yaml
```


```bash
vim
~/yahboom_ws/src/largemodel/config/HemiHex.yaml
```

Modify/Confirm llm_platform :

```bash
model_service
:
#Model server node parameters
ros__parameters
:
language
:
'zh'
#Large Model Interface Language
useolinetts
:
True
#This item is invalid in text mode and can be ignored
​
# Large model configuration
llm_platform
:
'ollama'
#Key: Make sure it's 'ollama'
regional_setting
:
"China"
```


```bash
model_service
:
#Model server node parameters
```

```bash
ros__parameters
:
```

```bash
language
:
'zh'
#Large Model Interface Language
```

```bash
useolinetts
:
True
#This item is invalid in text mode and can be ignored
```

```bash
​
```

```bash
# Large model configuration
```

```bash
llm_platform
:
'ollama'
#Key: Make sure it's 'ollama'
```

```bash
regional_setting
:
"China"
```

#### 3.1.2 Configuration model interface( large_model_interface.yaml )

This file defines which visual model to use when the platform is selected as ollama .

```bash

vim ~/yahboom_ws/src/largemodel/config/large_model_interface.yaml
```


```bash
vim ~/yahboom_ws/src/largemodel/config/large_model_interface.yaml
```

```bash
#.....
## 离线大模型 (Offline Large Language Models)
# Ollama Configuration
ollama_host:
"http://localhost:11434"
# Ollama server address
ollama_model:
"llava"
# Key: Change this to the multimodal model you downloaded, such as "llava"
#.....
```


```bash
#.....
```

```bash
## 离线大模型 (Offline Large Language Models)
```

```bash
# Ollama Configuration
```

```bash
ollama_host:
"http://localhost:11434"
# Ollama server address
```

```bash
ollama_model:
"llava"
# Key: Change this to the multimodal model you downloaded, such as "llava"
```

```bash
#.....
```

Note : Please make sure that the model specified in the configuration parameters (such as llava ) can handle multimodal input.

### 3.2 Starting and Testing the Function

Note: Due to performance limitations, this example cannot be run on the Jetson Orin Nano 4GB. To experience this function, please refer to the corresponding section in [Online Large Model (Voice Interaction)]

Start the largemodel main program : Open a terminal and run the following command:

```bash

ros2 launch largemodel largemodel_control.launch.py
```


```bash
ros2 launch largemodel largemodel_control.launch.py
```

After successful initialization, say the wake-up word and begin asking questions based on the current environment. Save the generated description of the environment as a text document.

Observe the results : In the first terminal running the main program, you will see log output indicating that the system receives the text command, invokes the aiagent tool, and then provides a prompt to the LLM. The LLM will analyze the detailed tool invocation steps. For example, the current question will invoke the seewhat tool to capture the image, which will then be parsed by the LLM. The parsed text will be saved in the ~/yahboom_ws/src/largemodel/resources_file/documents folder.
