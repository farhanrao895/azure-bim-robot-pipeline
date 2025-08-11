Phase 3 – Robotic Task Simulation with ROS2 & Gazebo
📌 Overview
This folder contains the Phase 3 deliverables for the Azure-Based BIM to Robot Simulation Pipeline.

Phase 3 demonstrates the complete integration of Azure PromptOrchestrator (from Phase 2) with ROS2 and Gazebo to enable LLM-driven multi-step construction task planning using Chain-of-Thought reasoning. The system simulates a two-robot construction workflow inside an IFC-aligned Digital Twin building with real-time task execution and telemetry feedback.

📋 Deliverables
Deliverable 1 – ROS2 Node to Simulate Task
File: phase3_prompt_orchestrator_executor.py (25 KB)
Description: Core ROS2 node that bridges Azure cloud intelligence with robotic simulation. It fetches Chain-of-Thought reasoning from Azure PromptOrchestrator and translates it into executable robot commands.

Key Features:

Connects to Azure PromptOrchestrator endpoint every 45 seconds

Processes 3-step Chain-of-Thought reasoning
(Site Analysis → Task Sequencing → Execution)

Controls 2 specialized construction robots (ground floor & upper floor)

Publishes ROS2 navigation goals (/move_base_simple/goal)

Generates real-time telemetry in Azure-compatible JSON format

Tracks reasoning history and task completion

Key Functions:

python
Copy
Edit
fetch_prompt_orchestration()     # Fetches LLM reasoning from Azure
process_reasoning_steps()         # Converts reasoning to structured tasks
execute_llm_driven_movements()    # Moves robots based on LLM tasks
publish_digital_twin_status()     # Sends telemetry to Azure
Deliverable 2 – Simulated Robot Workflow
File: enhanced_workflow_simulator.py (22 KB)
Description: Advanced workflow simulator that manages the entire construction process through 6 adaptive phases, responding dynamically to LLM reasoning updates.

Key Features:

6-phase construction workflow with reasoning-based adaptation

Subscribes to /llm/reasoning_steps for real-time updates

Coordinates multi-robot movements across floors

Implements safety verification and quality checkpoints

Generates workflow completion reports

Construction Phases:

Site Analysis – Building survey & accessibility checks

Foundation Preparation – Excavation & marking

Structural Assembly – Columns, beams, walls

Upper Floor Construction – Second floor parallel execution

Systems Integration – MEP installation

Quality Finishing – Final inspection & certification

📂 Supporting Files
Configuration Files

azure_config_advanced.json (3 KB) – Azure endpoints & building specs

chain_of_thought_history.json (189 KB) – Logged LLM reasoning steps

prompt_orchestrator_telemetry.json (2 KB) – Real-time telemetry

phase3_promptorchestrator_compliance.json (1 KB) – Compliance results

Simulation Files

phase3_ifc_digital_twin.sdf (17 KB) – Gazebo world with IFC building

phase3_advanced_task_executor.py (15 KB) – Alternative executor

verify_phase3_promptorchestrator.py (9 KB) – Compliance checker

Launch Scripts

launch_phase3_ifc_aligned.sh (2 KB) – Basic launcher

launch_phase3_promptorchestrator.sh (5 KB) – Full system launcher

🏗️ System Architecture
plaintext
Copy
Edit
Azure Cloud (Phase 2)                     Local Simulation (Phase 3)
┌─────────────────────┐                  ┌─────────────────────┐
│ PromptOrchestrator  │───HTTP/JSON──────▶│ ROS2 Task Executor │
│ (Chain-of-Thought)  │                  │ (Deliverable 1)     │
└─────────────────────┘                  └──────────┬──────────┘
                                                    │
                                                    ▼
┌─────────────────────┐                  ┌─────────────────────┐
│ Azure Digital Twin  │◀──Telemetry──────│ Workflow Simulator  │
│ (Building Model)    │                  │ (Deliverable 2)     │
└─────────────────────┘                  └──────────┬──────────┘
                                                    │
                                                    ▼
                                         ┌─────────────────────┐
                                         │ Gazebo Simulation   │
                                         │ (2 Robots, 5 Rooms) │
                                         └─────────────────────┘
⚙️ Installation & Setup
Prerequisites
bash
Copy
Edit
# ROS2 Humble
source /opt/ros/humble/setup.bash

# Gazebo Garden
sudo apt install gz-garden

# Python dependencies
pip3 install requests rclpy geometry-msgs std-msgs
🚀 Running the System
Method 1 – Using Launch Script (Recommended)

bash
Copy
Edit
./launch_phase3_promptorchestrator.sh
Method 2 – Manual Component Launch

bash
Copy
Edit
# Terminal 1: Gazebo
gz sim phase3_ifc_digital_twin.sdf

# Terminal 2: ROS2 executor
python3 phase3_prompt_orchestrator_executor.py

# Terminal 3: Workflow simulator
python3 enhanced_workflow_simulator.py
Method 3 – Minimal Test Run

bash
Copy
Edit
source /opt/ros/humble/setup.bash
pkill -f gazebo; pkill -f gz; pkill -f ros2
gz sim phase3_ifc_digital_twin.sdf &
sleep 10
python3 phase3_prompt_orchestrator_executor.py &
python3 enhanced_workflow_simulator.py &
wait
📊 Monitoring & Outputs
Real-Time Monitoring

bash
Copy
Edit
watch -n 1 cat prompt_orchestrator_telemetry.json
watch -n 1 cat chain_of_thought_history.json
tail -f reasoning_workflow_*.json
Generated Files

File	Description	Update Frequency
prompt_orchestrator_telemetry.json	Robot positions & task status	Every 10s
chain_of_thought_history.json	LLM reasoning steps	Every 45s
reasoning_workflow_*.json	Workflow reports	Per workflow
phase3_promptorchestrator_completion.json	Final report	On completion

🏢 Digital Twin Building Specs
Structure: Mixed-use Residential/Commercial

Total Area: 73 m²

Floors: 2 (Ground + Upper)

Total Spaces: 5

Ground Floor:

BEDROOM (01) – 15 m²

BEDROOM (02) – 15 m²

KITCHEN (102) – 12 m²

BATHROOM (103) – 6 m²

Upper Floor:

LOUNGE (104) – 25 m²

Robots:

azure_construction_robot_01 – Ground floor specialist

azure_construction_robot_02 – Upper floor specialist

🔗 Integration with Other Phases
Input from Phase 2:

PromptOrchestrator endpoint: /api/PromptOrchestrator

Chain-of-Thought reasoning (3 steps)

Structured JSON task plans

Output to Phase 4:

Real-time telemetry

Task completion status

Workflow metrics

👥 For Other Users
1️⃣ Prerequisites

ROS2 Humble + Gazebo Garden

Python 3.8+

(Optional) Azure OpenAI & Digital Twins

2️⃣ Setup

bash
Copy
Edit
pip install requests rclpy geometry-msgs std-msgs pyyaml
Update azure_config_advanced.json with Azure endpoints.

3️⃣ Run

bash
Copy
Edit
bash launch_phase3_promptorchestrator.sh
4️⃣ Stop

bash
Copy
Edit
pkill -f gazebo
pkill -f ros2
📌 Code Availability
This repository contains two full runnable codebases:

Phase 1–2 complete code → In Phase 1-2 folder, .py file creates everything for those phases.

Phase 3 complete code → In Phase 3 folder, .py file creates and runs the full simulation.

Each can be run independently in their respective folders.

📝 License & Attribution
Part of the Azure-Based BIM to Robot Simulation Pipeline proof-of-concept.

Technologies Used:

ROS2 Humble

Gazebo Garden

Azure PromptOrchestrator

Azure Digital Twins

Python 3.8+
