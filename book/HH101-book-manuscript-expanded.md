# HH-101 Expanded Book Draft (Chapters 1-2)

## Document Purpose
This file is a publish-ready expansion draft for:
- Chapter 1: Board Basics and System Preparation
- Chapter 2: Linux Basics for Embedded Development

It is intended to set the writing standard for the rest of the book:
- concept-first explanation
- practical action steps
- clear validation criteria
- troubleshooting guidance
- exercises for retention

---

# Part I - Foundations

## Chapter 1 - Board Basics and System Preparation

### Learning goals
By the end of this chapter, readers should be able to:
- explain what the Jetson Orin platform is and why it is used for edge AI;
- identify critical hardware interfaces for first-time bring-up;
- choose the correct setup path based on project risk and goals;
- install and verify a stable baseline environment.

### Prerequisites
- Jetson Orin Nano or Orin NX development board
- power adapter, display cable, USB Type-C cable, ethernet cable
- keyboard and mouse
- host PC or VM for SDK-related operations

### Estimated time
3-4 hours (first-time setup), 1.5-2 hours (repeat setup)

### Why this chapter matters
In embedded AI projects, setup quality determines the success of every chapter that follows. If the baseline system is unstable, later failures in OpenCV, ROS, or model execution will be harder to diagnose because root causes become mixed together.

This chapter establishes an operational baseline. The main objective is not only to boot the board, but to build a repeatable setup workflow with checkpoints and recovery logic. A reader who completes this chapter correctly should be able to recover from common startup and installation issues without trial-and-error guessing.

### 1.1 Understanding the role of Jetson in this course
Jetson is an edge AI platform: it combines CPU, GPU, and embedded interfaces in a low-power form factor. In this course, it acts as:
- the runtime platform for AI inference;
- the control platform for robotics software;
- the integration point for cameras, sensors, and networked services.

Compared with a laptop, Jetson introduces deployment realism. You work with power/thermal limits, constrained memory, and hardware interfaces that are common in real products. This is why setup discipline is critical from day one.

### 1.2 Hardware orientation before powering on
Before connecting cables, the reader should map the board interfaces by function:
- power and boot control;
- display output;
- USB host and USB device ports;
- camera interfaces and expansion headers;
- storage interfaces (NVMe slots).

This prevents a common beginner mistake: mixing interfaces during setup and then debugging software for what is actually a physical wiring issue.

Recommended pre-power checklist:
1. confirm adapter spec and cable integrity;
2. confirm display path and input source;
3. confirm USB Type-C data path to host/VM if needed;
4. confirm ethernet availability for package installs.

### 1.3 Setup decision framework (the most important decision)
Readers should not jump directly into flashing. The first decision should be:
- keep factory image and install required components; or
- reflash official clean system; or
- use SUPER workflow.

Decision guidance:
- choose factory-image path if the board boots and you are learning fundamentals;
- choose reflash only when system state is uncertain or a clean reset is required;
- choose SUPER workflow only when your exact downstream scenario requires it.

This decision protects progress and reduces unnecessary data loss.

### 1.4 Safe workspace rules before system writes
Any workflow involving recovery mode, flashing, or partition updates should follow strict preconditions:
- stable power and network;
- no loose USB passthrough in VM;
- no parallel heavy downloads on host;
- clear record of target storage device.

A practical habit is to keep a short setup log:
- board model;
- selected workflow;
- software version targets;
- timestamped checkpoints.

When something fails, the log makes rollback and retry systematic.

### 1.5 Installing component environment with SDK workflow
For beginner-first setup, use the normal boot path and install missing Jetson components with SDK Manager.

Operational intent:
- avoid rewriting system image when not required;
- install only what is needed for current module goals;
- keep installation reproducible by recording selected package options.

Execution pattern:
1. boot board normally and verify desktop access;
2. connect board to host/VM with Type-C;
3. attach device correctly in VM passthrough (if VM is used);
4. select exact target model in SDK Manager;
5. run install and monitor for connectivity interruptions;
6. reboot and validate runtime tools.

The key teaching point is that setup is not a single click. It is a monitored process with checkpoints.

### 1.6 Verifying system health and performance baseline
A proper verification stage should cover:
- package/state validation;
- system telemetry;
- performance mode.

Suggested validation outputs:
- package manager completes update and upgrade without unresolved dependencies;
- `jtop` starts and reports expected hardware metrics;
- power model is set intentionally for the board model;
- clocks/power settings are understood, not copied blindly.

Readers should be taught that "it runs" is not enough. A baseline must be measurable.

### 1.7 Common failure scenarios and recovery playbook
Most failures are repeatable and solvable. This section should train response patterns:

Scenario A: board not detected in host/VM  
Likely causes: cable quality, wrong mode, VM device attach state.  
Response: confirm physical cable, confirm mode, reattach USB device, recheck detection.

Scenario B: installation step fails mid-process  
Likely causes: transient network issues, VM instability, package timeout.  
Response: reboot board, restart installer from failed step, verify storage and credentials.

Scenario C: boot issues after writes  
Likely causes: wrong target, mismatched image/workflow, incomplete write.  
Response: re-enter correct mode, re-validate target selection, repeat with stable power.

The reader benefit is confidence. A failed setup becomes a known workflow, not a dead end.

### 1.8 Chapter summary
This chapter established the core setup philosophy of the book:
- choose the least risky valid path first;
- treat hardware and software setup as one system;
- verify baseline health before moving to higher-level modules.

If this foundation is stable, all later chapters become faster and more reliable.

### Validation checklist (chapter completion)
- Board boots reliably into expected system state.
- Chosen setup path is documented with rationale.
- Core environment install completed successfully.
- System telemetry tools report expected values.
- Recovery procedure is understood and testable.

### Exercises
#### Core exercise
Create a one-page setup runbook that includes:
- your board model;
- selected setup path and why;
- a 10-step bring-up checklist;
- three failure cases and responses.

#### Extension exercise
Repeat setup verification on a second day and compare:
- time to complete;
- number of issues encountered;
- number of manual interventions.

Use this to refine your runbook.

### Source lesson mapping
- `docs/hh101/HH-101/02 - Board Basics Tasks/00-module-overview.md`
- `docs/hh101/HH-101/02 - Board Basics Tasks/01-Jetson-board-introduction.md`
- `docs/hh101/HH-101/02 - Board Basics Tasks/02-choose-setup-path.md`
- `docs/hh101/HH-101/02 - Board Basics Tasks/03-installing-jetson-environment.md`
- `docs/hh101/HH-101/02 - Board Basics Tasks/04-write-jetson-original-system.md`

---

## Chapter 2 - Linux Basics for Embedded Development

### Learning goals
By the end of this chapter, readers should be able to:
- use Linux command-line tools confidently on Jetson;
- set up robust remote workflows (SSH, VNC, transfer);
- maintain a repeatable development environment for future modules.

### Prerequisites
- Chapter 1 baseline completed
- network connectivity between host and Jetson

### Estimated time
4-6 hours (first pass), 2-3 hours (review pass)

### Why this chapter matters
A robotics or AI developer spends more time operating systems than writing model code. Linux proficiency directly impacts debugging speed, reproducibility, and project reliability.

This chapter converts "command familiarity" into operational competence. The reader should leave with an environment that supports real development loops: edit, run, inspect, diagnose, and recover.

### 2.1 Adopting the Linux troubleshooting mindset
Linux expertise starts with process, not memorization:
- inspect state before changing state;
- capture command outputs for reproducibility;
- isolate one variable at a time during debugging;
- keep working notes of what changed.

Readers should be encouraged to treat terminal history as a lab notebook.

### 2.2 Shell fundamentals for project execution
Core shell tasks should be framed around project workflows:
- navigating workspaces and package directories;
- inspecting files and permissions;
- editing configs safely;
- running scripts with explicit environment context.

A useful teaching point is to emphasize predictable paths and clear naming. Many build failures come from running commands in the wrong directory.

### 2.3 Package management and system hygiene
Package operations should be taught as controlled maintenance:
- update indexes intentionally;
- upgrade with awareness of dependency impact;
- install only required tools for current goals;
- avoid mixing conflicting installation methods unnecessarily.

Readers should learn to verify package states and avoid "blind reinstall loops."

### 2.4 Network diagnostics for embedded systems
Before remote tooling, validate network fundamentals:
- IP discovery;
- route reachability;
- DNS and outbound connectivity;
- latency stability for long-running installs.

Practical insight: many "tool failures" are network-state failures.

### 2.5 SSH as primary remote control channel
SSH should be the default remote interface because it is scriptable, reliable, and low overhead.

Best-practice discussion:
- stable host aliases;
- key-based auth when appropriate;
- explicit config entries for repeatability;
- log capture for remote runs.

Teach readers to run most setup and diagnostics over SSH first, then use GUI only when necessary.

### 2.6 VNC as secondary GUI access path
VNC is valuable for GUI-dependent tasks, but it should be treated as complementary, not primary. Readers should learn:
- expected performance limits;
- display/session assumptions;
- failure recovery when GUI path breaks.

A resilient workflow always keeps SSH available as fallback.

### 2.7 File transfer and workspace synchronization
File movement should be standardized:
- choose transfer method by file size and frequency;
- preserve permissions when needed;
- avoid hidden drift between host and target copies.

A practical recommendation is to define one canonical workspace path and avoid duplicate project copies unless required.

### 2.8 Development tooling: VS Code and system insight tools
The goal of tooling is to reduce context switching:
- code editing and terminal access in one workflow;
- repeatable launch commands;
- clear monitoring of system load and thermal/power behavior.

Tools like `jtop` should be integrated into routine checks, especially before and during heavier vision/model workloads.

### 2.9 Operational patterns for long-term stability
Readers should adopt sustainable habits:
- document environment assumptions;
- keep small reusable scripts for repeated checks;
- snapshot key configs before major changes;
- validate environment after any major update.

These habits reduce future integration risk when moving into ROS and AI model modules.

### 2.10 Chapter summary
This chapter built the operational layer required for the rest of the course:
- command-line fluency;
- robust remote workflows;
- repeatable tooling environment.

With this foundation, later robotics and AI chapters can focus on system behavior rather than setup friction.

### Validation checklist (chapter completion)
- SSH connection works consistently from host.
- VNC session is available and usable.
- File transfer is reliable both directions.
- Editor workflow can run commands on target without manual friction.
- Basic system diagnostics can be executed and interpreted.

### Exercises
#### Core exercise
Create a "daily startup script" that:
- checks network and disk;
- confirms key services;
- prints quick system telemetry.

Run it for three sessions and record observations.

#### Extension exercise
Set up two remote workflows:
- SSH-only workflow;
- SSH + VNC workflow.

Compare reliability, speed, and suitability for different tasks.

### Source lesson mapping
- `docs/hh101/HH-101/03 - Linux Basics/00-module-overview.md`
- `docs/hh101/HH-101/03 - Linux Basics/01-linux-basics.md`
- `docs/hh101/HH-101/03 - Linux Basics/02-network-configuration.md`
- `docs/hh101/HH-101/03 - Linux Basics/03-ssh-remote-login-3-3.md`
- `docs/hh101/HH-101/03 - Linux Basics/04-vnc-remote-control-3-4.md`
- `docs/hh101/HH-101/03 - Linux Basics/05-remote-file-transfer-03-linux-basics-3-5.md`
- `docs/hh101/HH-101/03 - Linux Basics/06-jtop-tool-03-linuxbasics-3-6.md`
- `docs/hh101/HH-101/03 - Linux Basics/08-vscode-usage-03-linuxbasics-3-8.md`

---

## Chapter 3 - GPIO and Hardware I/O Basics

### Learning goals
By the end of this chapter, readers should be able to:
- explain how GPIO is used in robotics and embedded AI systems;
- perform reliable digital input and output tests;
- validate and troubleshoot a basic I2C communication path.

### Prerequisites
- Chapter 1 and Chapter 2 completed
- known-good wiring setup and power stability

### Estimated time
2-3 hours

### Why this chapter matters
GPIO is the first point where software behavior is constrained by physical reality. If readers only practice high-level software and ignore I/O fundamentals, they will struggle later with sensors, actuators, and robot control reliability.

### 3.1 Expanded discussion
Hardware I/O introduces timing, voltage, and signal integrity concerns that do not exist in purely software exercises. This is why simple, deterministic tests are essential before integrating with larger robotics stacks.

Readers should understand that most "GPIO bugs" are integration bugs:
- incorrect pin mapping;
- wrong pull-up or pull-down assumptions;
- overlooked wiring constraints;
- mismatched bus addressing on I2C devices.

A good teaching strategy is to start with single-pin tests, then scale to bus-level communication.

### 3.2 Practical workflow
1. Confirm pinout mapping and safety constraints.
2. Install and verify GPIO library readiness.
3. Run digital output test with visible indicator (LED or equivalent).
4. Run digital input test with controlled signal source.
5. Scan I2C bus and verify expected device addresses.
6. Execute one read/write sanity test for an I2C peripheral.

### Validation checklist
- Pin numbering scheme is documented and consistent.
- Digital output toggles as expected.
- Digital input values are stable and explainable.
- I2C scan detects expected device address.
- Basic I2C transaction succeeds without intermittent failures.

### Exercises
#### Core exercise
Build a minimal "sensor heartbeat" script that reads one GPIO input every second and logs value plus timestamp.

#### Extension exercise
Add fault detection:
- detect disconnected device condition;
- emit structured warning output;
- recover gracefully without crashing.

### Source lesson mapping
- `docs/hh101/HH-101/04 - GPIO control course/00-module-overview.md`
- `docs/hh101/HH-101/04 - GPIO control course/01-gpio-description-04-gpiocontrolcourse-4-1.md`
- `docs/hh101/HH-101/04 - GPIO control course/02-gpio-library-installation-04-gpiocontrolcourse-4-2.md`
- `docs/hh101/HH-101/04 - GPIO control course/03-gpio-reading-04-gpiocontrolcourse-4-3.md`
- `docs/hh101/HH-101/04 - GPIO control course/04-i2c-communication-04-gpiocontrolcourse-4-4.md`

---

# Part II - Vision Core

## Chapter 4 - Camera Bring-Up and Vision Foundations

### Learning goals
By the end of this chapter, readers should be able to:
- configure and validate CSI and USB camera inputs;
- compare camera source behavior and choose suitable pipelines;
- use notebook-based workflows for rapid visual experimentation.

### Prerequisites
- foundational setup complete
- camera modules available and physically connected

### Estimated time
3-4 hours

### Why this chapter matters
Vision projects often fail early because camera assumptions are never validated. A model cannot compensate for unstable input streams, wrong formats, or mismatched resolutions.

### 4.1 Expanded discussion
The camera layer should be treated as system infrastructure, not a quick pre-step. Readers must verify:
- source availability;
- format compatibility;
- frame stability over time;
- acceptable latency and frame rate.

A beginner should first establish "known good frames" before any AI inference. This isolates image acquisition from model complexity.

### 4.2 Practical workflow
1. Verify camera hardware detection and device nodes.
2. Preview CSI camera stream.
3. Preview USB camera stream.
4. Compare resolution, FPS, and stability.
5. Set up Jupyter workflow for quick frame experiments.

### Validation checklist
- CSI preview runs without frame drops.
- USB preview runs without pipeline errors.
- Chosen default camera path is documented.
- Notebook workflow can capture and display frames.

### Exercises
#### Core exercise
Capture one reference frame set from each camera type and document differences in quality and latency.

#### Extension exercise
Create a simple camera switch script that chooses CSI or USB source via argument and outputs consistent frame metadata.

### Source lesson mapping
- `docs/hh101/HH-101/05 - Vision Basic Course/00-module-overview.md`
- `docs/hh101/HH-101/05 - Vision Basic Course/01-csi-camera-preview-05-visionbasiccourse-5-1.md`
- `docs/hh101/HH-101/05 - Vision Basic Course/02-usb-camera-preview-05-visionbasiccourse-5-2.md`
- `docs/hh101/HH-101/05 - Vision Basic Course/03-use-jupyter-lab-05-visionbasiccourse-5-3.md`
- `docs/hh101/HH-101/05 - Vision Basic Course/04-use-jetcam-05-visionbasiccourse-5-4.md`

---

## Chapter 5 - OpenCV for Embedded Computer Vision

### Learning goals
By the end of this chapter, readers should be able to:
- perform core OpenCV image operations with confidence;
- build reusable scripts for transformation and annotation tasks;
- integrate OpenCV into camera-driven pipelines.

### Prerequisites
- camera validation completed
- basic Python development workflow available

### Estimated time
6-8 hours

### Why this chapter matters
OpenCV is the connective tissue between raw sensor input and AI model pipelines. It handles preprocessing, geometric operations, annotation, and many practical utilities needed in production workflows.

### 5.1 Expanded discussion
Readers should understand where OpenCV is most effective:
- preparing data before model inference;
- cleaning and normalizing frames;
- implementing fast classical operations;
- visualizing and validating results.

OpenCV should be taught with performance awareness. On embedded devices, unnecessary copies and oversized frames have a direct runtime cost.

### 5.2 Practical workflow
1. Image read and save operations.
2. Pixel-level modifications and resizing.
3. Cropping and translation.
4. Mirroring, grayscale, thresholding, edges.
5. Drawing lines, shapes, and text overlays.
6. USB/CSI live camera preview through OpenCV.

### Validation checklist
- Each transformation produces expected visual output.
- Camera processing loop remains stable over sustained runtime.
- Annotation overlays align with image coordinates.
- Script structure is reusable across input sources.

### Exercises
#### Core exercise
Build a single command-line tool with flags for:
- input file or camera source;
- selected transform sequence;
- output destination.

#### Extension exercise
Measure and compare runtime for two resolution settings and summarize tradeoffs.

### Source lesson mapping
- `docs/hh101/HH-101/06 - OpenCV/00-module-overview.md`
- `docs/hh101/HH-101/06 - OpenCV/*.md`

---

# Part III - Advanced Vision And Model Workflows

## Chapter 6 - Deep Learning Vision on Jetson

### Learning goals
By the end of this chapter, readers should be able to:
- set up advanced vision frameworks on Jetson;
- run detector, segmenter, and pose pipelines;
- reason about conversion and deployment tradeoffs.

### Prerequisites
- stable OpenCV and camera workflows
- baseline understanding of model inference concepts

### Estimated time
10-14 hours

### Why this chapter matters
This chapter turns the platform into a practical AI workstation. It also introduces deployment thinking: not only "can it run?" but "can it run reliably and efficiently on edge hardware?"

### 6.1 Expanded discussion
Framework selection should be goal-driven:
- TensorFlow and PyTorch for flexibility and experimentation;
- YOLO pipelines for fast task-focused implementation;
- DeepStream for scalable video analytics flows;
- MediaPipe for graph-based multimodal pipelines.

Readers should understand model lifecycle stages:
- environment setup;
- baseline inference;
- optimization and conversion;
- production validation.

### 6.2 Practical workflow
1. Install and validate TF/PyTorch runtimes.
2. Run baseline inference with known models.
3. Execute YOLO tasks: detection, segmentation, pose, classification.
4. Convert models for deployment workflows.
5. Run DeepStream and MediaPipe module examples.

### Validation checklist
- Framework imports and runtime checks pass.
- Model outputs are logically correct.
- Conversion outputs load and run.
- Pipeline latency and resource usage are measured.

### Exercises
#### Core exercise
Run one use case in two frameworks and compare setup complexity, runtime, and output quality.

#### Extension exercise
Create a short benchmark report:
- model type;
- input size;
- measured throughput or latency;
- observed bottlenecks.

### Source lesson mapping
- `docs/hh101/HH-101/07 - Advanced Vision/00-module-overview.md`
- `docs/hh101/HH-101/07 - Advanced Vision/**/*.md`

---

# Part IV - Platform Engineering And Robotics Middleware

## Chapter 7 - Docker for Reproducible Embedded Development

### Learning goals
By the end of this chapter, readers should be able to:
- explain containerization benefits and limits on Jetson;
- use Docker for repeatable development environments;
- manage container lifecycle and interaction patterns.

### Prerequisites
- Linux command-line familiarity
- network and package management stable

### Estimated time
2-3 hours

### Why this chapter matters
As projects scale, environment drift becomes a major source of failure. Docker reduces this by making dependencies explicit and reproducible across machines and contributors.

### 7.1 Expanded discussion
Readers should understand Docker in practical terms:
- image is immutable baseline;
- container is runnable instance;
- volume and bind mount define data ownership boundaries.

On Jetson-class systems, container workflows must also account for device access and GPU compatibility.

### 7.2 Practical workflow
1. Install Docker and verify daemon status.
2. Pull and run a test image.
3. Execute common command patterns.
4. Use mounts and environment variables safely.
5. Start/stop/inspect containers and clean up resources.

### Validation checklist
- Docker service starts reliably.
- Containers can run and exit cleanly.
- Data persistence behavior is understood.
- Core commands are documented in a personal cheat sheet.

### Exercises
#### Core exercise
Create a minimal containerized dev environment and persist one workspace folder via mount.

#### Extension exercise
Write a short run command template that standardizes:
- container name;
- mounted paths;
- networking mode;
- restart policy.

### Source lesson mapping
- `docs/hh101/HH-101/08 - Docker/00-module-overview.md`
- `docs/hh101/HH-101/08 - Docker/*.md`

---

## Chapter 8 - ROS1 Core Communication Patterns

### Learning goals
By the end of this chapter, readers should be able to:
- build and inspect ROS1 node communication patterns;
- implement topic and service workflows;
- use custom messages and basic TF workflows.

### Prerequisites
- Chapter 2 Linux workflow complete
- workspace build toolchain available

### Estimated time
5-7 hours

### Why this chapter matters
ROS1 remains a foundational robotics learning environment. The architecture of nodes, topics, and services is essential context even when later moving to ROS2.

### 8.1 Expanded discussion
Readers should visualize ROS as a graph of responsibilities:
- nodes own behavior;
- topics move streaming data;
- services handle request/response interactions.

When communication issues appear, the fix is usually clearer if the graph is drawn first and code is checked second.

### 8.2 Practical workflow
1. Create and structure ROS1 workspace.
2. Implement publisher and subscriber pair.
3. Implement service client and server.
4. Create and use custom message/service types.
5. Inspect communication graph and TF behavior.

### Validation checklist
- Nodes register and communicate as expected.
- Topic data appears with correct type and rate.
- Service call returns expected output.
- Custom message/service build and runtime behavior are correct.

### Exercises
#### Core exercise
Build a two-node ROS1 mini system where one node publishes sensor-style data and another logs filtered results.

#### Extension exercise
Add a service endpoint that toggles behavior mode in the subscriber node.

### Source lesson mapping
- `docs/hh101/HH-101/09 - ROS1/00-module-overview.md`
- `docs/hh101/HH-101/09 - ROS1/*.md`

---

## Chapter 9 - ROS2 Distributed Robotics Workflows

### Learning goals
By the end of this chapter, readers should be able to:
- develop ROS2 nodes and communication patterns;
- understand DDS and domain isolation concepts;
- use launch, bag, RViz2, URDF, Gazebo, and TF2 in workflows.

### Prerequisites
- ROS1 concepts helpful but not mandatory
- stable development environment from earlier chapters

### Estimated time
10-14 hours

### Why this chapter matters
ROS2 is central to modern robotics deployments. This chapter introduces not only APIs but also operational behavior in distributed systems.

### 9.1 Expanded discussion
ROS2 adds significant architectural considerations:
- middleware behavior through DDS;
- domain and network boundaries;
- launch-based system orchestration;
- richer tooling for introspection and replay.

Readers should move from "run this node" to "design and operate a distributed graph."

### 9.2 Practical workflow
1. Install and validate ROS2 environment.
2. Build core nodes/topics/services.
3. Configure parameters and launch files.
4. Record and replay workflows with bag tools.
5. Visualize state and transforms with RViz2.
6. Build simple simulation path with URDF and Gazebo.

### Validation checklist
- ROS2 workspace builds cleanly.
- Nodes communicate in expected domain.
- Launch configurations start full stacks reproducibly.
- TF2 frames are coherent and inspectable.

### Exercises
#### Core exercise
Create a launch file that starts three coordinated nodes and validate startup order and parameters.

#### Extension exercise
Record one bag file and replay it to verify deterministic behavior in downstream nodes.

### Source lesson mapping
- `docs/hh101/HH-101/10 - ROS2/00-module-overview.md`
- `docs/hh101/HH-101/10 - ROS2/*.md`

---

# Part V - AI Application Layers

## Chapter 10 - Offline AI Models on Edge Devices

### Learning goals
By the end of this chapter, readers should be able to:
- deploy local model runtimes for text and multimodal tasks;
- evaluate model choice against device resource limits;
- implement local-first AI application patterns.

### Prerequisites
- stable OS and tooling baseline
- familiarity with model inference concepts

### Estimated time
12-16 hours

### Why this chapter matters
Offline AI is critical for privacy, cost control, and low-latency local operation. This chapter helps readers build practical systems that do not depend on constant cloud connectivity.

### 10.1 Expanded discussion
Offline model workflows require explicit resource planning:
- model size vs latency tradeoff;
- memory pressure and concurrent workloads;
- prompt/context size constraints;
- fallback behavior when hardware limits are reached.

Readers should understand that robust local AI design is a systems problem, not just a model selection problem.

### 10.2 Practical workflow
1. Install local runtime and management tools.
2. Load and test selected models.
3. Run baseline prompt and multimodal workflows.
4. Measure performance and refine model choice.
5. Integrate local model calls into application flow.

### Validation checklist
- Models load consistently and respond reliably.
- Runtime behavior is stable under repeated queries.
- Resource use remains within safe limits.
- Failure handling is defined for overload scenarios.

### Exercises
#### Core exercise
Compare two local models for one fixed task and report quality, speed, and resource impact.

#### Extension exercise
Implement a fallback chain:
- preferred model;
- lighter backup model;
- clear error output if both fail.

### Source lesson mapping
- `docs/hh101/HH-101/11 - Offline AI Model/00-module-overview.md`
- `docs/hh101/HH-101/11 - Offline AI Model/*.md`

---

## Chapter 11 - Online AI Models and API-Orchestrated Features

### Learning goals
By the end of this chapter, readers should be able to:
- integrate cloud model APIs safely and consistently;
- build multimodal and voice application flows;
- engineer around latency and service constraints.

### Prerequisites
- understanding of local model workflows (Chapter 10 helpful)
- secure handling of API credentials

### Estimated time
8-12 hours

### Why this chapter matters
Cloud APIs provide powerful capabilities but add external dependencies. Engineering quality depends on robust request handling, safe key management, and clear fallback logic.

### 11.1 Expanded discussion
The core of online model integration is interface reliability:
- timeouts and retries;
- structured response parsing;
- observability and logging;
- version drift management.

Readers should treat model APIs like any production dependency with explicit contracts and guardrails.

### 11.2 Practical workflow
1. Configure provider credentials and endpoints.
2. Validate basic request and response path.
3. Build multimodal and voice interaction scenarios.
4. Add error handling, retry strategy, and logging.
5. Evaluate cost/latency tradeoffs for target use case.

### Validation checklist
- Auth and request flow are stable.
- Response parsing is deterministic for expected tasks.
- Error paths are handled without process failure.
- Usage limits and latency impacts are monitored.

### Exercises
#### Core exercise
Implement one feature using online API with:
- structured input template;
- structured output validation;
- retry and timeout policy.

#### Extension exercise
Design a hybrid mode that switches to offline behavior when API fails or exceeds latency threshold.

### Source lesson mapping
- `docs/hh101/HH-101/12 - Online AI Model/00-module-overview.md`
- `docs/hh101/HH-101/12 - Online AI Model/*.md`

---

# Part VI - Isaac ROS Acceleration

## Chapter 12 - NVIDIA Isaac ROS Perception and SLAM Pipelines

### Learning goals
By the end of this chapter, readers should be able to:
- explain Isaac ROS value in accelerated robotics pipelines;
- set up Isaac ROS environments and run reference workflows;
- reason about integration points for perception and SLAM.

### Prerequisites
- ROS2 familiarity (Chapter 9)
- vision and model basics from earlier chapters

### Estimated time
10-14 hours

### Why this chapter matters
Isaac ROS is where optimized robotics perception workflows become practical on NVIDIA hardware. This chapter helps readers connect high-level robotics goals to accelerated runtime components.

### 12.1 Expanded discussion
Isaac ROS should be framed as a performance-enabling ecosystem:
- optimized components reduce integration effort;
- throughput and latency improvements support real-time constraints;
- interoperability with ROS2 patterns keeps system design coherent.

Readers should also understand the boundary between packaged acceleration and custom system integration responsibilities.

### 12.2 Practical workflow
1. Prepare Isaac ROS environment and dependencies.
2. Run baseline perception examples.
3. Execute depth/segmentation/detection flows.
4. Explore mapping and visual SLAM examples.
5. Validate end-to-end integration behavior in a ROS2 graph.

### Validation checklist
- Isaac ROS components start and run predictably.
- Perception outputs are interpretable and usable downstream.
- SLAM or mapping examples produce coherent results.
- Integration points with ROS2 tooling remain debuggable.

### Exercises
#### Core exercise
Run one Isaac ROS perception pipeline and document:
- data input path;
- processing graph;
- output topic(s);
- measured runtime behavior.

#### Extension exercise
Design a mini architecture diagram that combines:
- sensor input;
- Isaac ROS perception stage;
- decision node;
- output/control stage.

### Source lesson mapping
- `docs/hh101/HH-101/13 - NVIDIA Isaac ROS Course/00-module-overview.md`
- `docs/hh101/HH-101/13 - NVIDIA Isaac ROS Course/*.md`

---

## PDF Export
Use the same conversion flow as the main manuscript:

```powershell
pandoc book/HH101-book-manuscript-expanded.md `
  -o book/HH101-book-manuscript-expanded.pdf `
  --toc `
  --number-sections `
  --pdf-engine=xelatex
```
