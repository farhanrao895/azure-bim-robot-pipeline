markdown# 🤖 Phase 3 – Robotic Task Simulation with ROS2 & Gazebo

[![ROS2](https://img.shields.io/badge/ROS2-Humble-22314E?logo=ros)](https://docs.ros.org/)
[![Gazebo](https://img.shields.io/badge/Gazebo-Garden-orange)](https://gazebosim.org/)
[![Python](https://img.shields.io/badge/Python-3.8+-3776AB?logo=python)](https://www.python.org/)
[![Azure](https://img.shields.io/badge/Azure-Integrated-0078D4?logo=microsoft-azure)](https://azure.microsoft.com/)

## 📌 Overview

This folder contains the **Phase 3 deliverables** for the **Azure-Based BIM to Robot Simulation Pipeline**.

Phase 3 demonstrates the complete integration of **Azure PromptOrchestrator** (from Phase 2) with **ROS2** and **Gazebo** to enable **LLM-driven multi-step construction task planning** using **Chain-of-Thought reasoning**. The system simulates a **two-robot construction workflow** inside an **IFC-aligned Digital Twin building** with real-time task execution and telemetry feedback.

---

## 📋 Deliverables

### **🔹 Deliverable 1 – ROS2 Node to Simulate Task**

| Property | Details |
|----------|---------|
| **File** | `phase3_prompt_orchestrator_executor.py` |
| **Size** | 25 KB |
| **Purpose** | Bridges Azure cloud intelligence with robotic simulation |

**Description:**  
Core ROS2 node that fetches **Chain-of-Thought reasoning** from Azure PromptOrchestrator and translates it into executable robot commands.

**Key Features:**
- ⏱️ Connects to Azure PromptOrchestrator every **45 seconds**
- 🧠 Processes **3-step Chain-of-Thought reasoning**:  
  `Site Analysis → Task Sequencing → Execution`
- 🤖 Controls **2 specialized construction robots** (ground & upper floor)
- 📡 Publishes ROS2 navigation goals (`/move_base_simple/goal`)
- 📊 Generates real-time telemetry in Azure-compatible JSON format
- 📝 Tracks reasoning history and task completion

**Key Functions:**
```python
fetch_prompt_orchestration()     # Fetches LLM reasoning from Azure
process_reasoning_steps()         # Converts reasoning to structured tasks
execute_llm_driven_movements()    # Moves robots based on LLM tasks
publish_digital_twin_status()     # Sends telemetry to Azure
🔹 Deliverable 2 – Simulated Robot Workflow
PropertyDetailsFileenhanced_workflow_simulator.pySize22 KBPurposeManages 6-phase adaptive construction workflow
Description:
Advanced workflow simulator that manages the entire construction process through adaptive phases, responding dynamically to LLM reasoning updates.
Key Features:

🔄 6-phase construction workflow with reasoning-based adaptation
📡 Subscribes to /llm/reasoning_steps for real-time updates
🏗️ Coordinates multi-robot movements across floors
✅ Implements safety verification and quality checkpoints
📈 Generates workflow completion reports

Construction Phases:
PhaseDescription1️⃣ Site AnalysisBuilding survey & accessibility checks2️⃣ Foundation PreparationExcavation & marking3️⃣ Structural AssemblyColumns, beams, walls4️⃣ Upper Floor ConstructionSecond floor parallel execution5️⃣ Systems IntegrationMEP installation6️⃣ Quality FinishingFinal inspection & certification

📂 Supporting Files
Configuration Files
FileSizeDescriptionazure_config_advanced.json3 KBAzure endpoints & building specschain_of_thought_history.json189 KBLogged LLM reasoning stepsprompt_orchestrator_telemetry.json2 KBReal-time telemetryphase3_promptorchestrator_compliance.json1 KBCompliance results
Simulation Files
FileSizeDescriptionphase3_ifc_digital_twin.sdf17 KBGazebo world with IFC buildingphase3_advanced_task_executor.py15 KBAlternative executorverify_phase3_promptorchestrator.py9 KBCompliance checker
Launch Scripts
FileSizeDescriptionlaunch_phase3_ifc_aligned.sh2 KBBasic launcherlaunch_phase3_promptorchestrator.sh5 KBFull system launcher

🏗️ System Architecture
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
bash# ROS2 Humble
source /opt/ros/humble/setup.bash

# Gazebo Garden
sudo apt install gz-garden

# Python dependencies
pip3 install requests rclpy geometry-msgs std-msgs

🚀 Running the System
Method 1 – Using Launch Script (Recommended)
bash./launch_phase3_promptorchestrator.sh
Method 2 – Manual Component Launch
bash# Terminal 1: Gazebo
gz sim phase3_ifc_digital_twin.sdf

# Terminal 2: ROS2 executor
python3 phase3_prompt_orchestrator_executor.py

# Terminal 3: Workflow simulator
python3 enhanced_workflow_simulator.py
Method 3 – Minimal Test Run
bashsource /opt/ros/humble/setup.bash
pkill -f gazebo; pkill -f gz; pkill -f ros2

gz sim phase3_ifc_digital_twin.sdf &
sleep 10
python3 phase3_prompt_orchestrator_executor.py &
python3 enhanced_workflow_simulator.py &
wait

📊 Monitoring & Outputs
Real-Time Monitoring
bash# Watch telemetry updates
watch -n 1 cat prompt_orchestrator_telemetry.json

# Monitor reasoning history
watch -n 1 cat chain_of_thought_history.json

# Check workflow progress
tail -f reasoning_workflow_*.json
Generated Files
FileDescriptionUpdate Frequencyprompt_orchestrator_telemetry.jsonRobot positions & task statusEvery 10schain_of_thought_history.jsonLLM reasoning stepsEvery 45sreasoning_workflow_*.jsonWorkflow reportsPer workflowphase3_promptorchestrator_completion.jsonFinal reportOn completion

🏢 Digital Twin Building Specifications
PropertyValueStructureMixed-use Residential/CommercialTotal Area73 m²Floors2 (Ground + Upper)Total Spaces5
Floor Layout
Ground Floor (Elevation: 0.0m)

🛏️ BEDROOM (01) – 15 m²
🛏️ BEDROOM (02) – 15 m²
🍳 KITCHEN (102) – 12 m²
🚿 BATHROOM (103) – 6 m²

Upper Floor (Elevation: 3.5m)

🛋️ LOUNGE (104) – 25 m²

Construction Robots

🔵 azure_construction_robot_01 – Ground floor specialist
🔴 azure_construction_robot_02 – Upper floor specialist


🔗 Integration with Other Phases
📥 Input from Phase 2

PromptOrchestrator endpoint: /api/PromptOrchestrator
Chain-of-Thought reasoning (3 steps)
Structured JSON task plans with floor assignments
Safety protocols and dependencies

📤 Output to Phase 4

Real-time telemetry data
Task completion status
Workflow progress metrics
System performance logs


👥 For Other Users
1️⃣ Prerequisites

✅ ROS2 Humble + Gazebo Garden
✅ Python 3.8+
⭕ (Optional) Azure OpenAI & Digital Twins

2️⃣ Setup
bash# Install dependencies
pip install requests rclpy geometry-msgs std-msgs pyyaml

# Configure Azure endpoints
nano azure_config_advanced.json
3️⃣ Run
bashbash launch_phase3_promptorchestrator.sh
4️⃣ Stop
bashpkill -f gazebo
pkill -f ros2

📌 Code Availability
This repository contains full runnable codebases:
ComponentLocationDescriptionPhase 3 Complete CodePhase3_Complete_Code.pyCreates and runs the full simulationIndividual ScriptsPhase 3 folderEach component can be run independently

🛠️ Technologies Used

ROS2 Humble – Robot Operating System
Gazebo Garden – 3D simulation environment
Azure PromptOrchestrator – AI task planning
Azure Digital Twins – Cloud telemetry
Python 3.8+ – Implementation language


📝 License & Attribution
Part of the Azure-Based BIM to Robot Simulation Pipeline proof-of-concept.

🎯 Summary
Phase 3 successfully demonstrates:

✅ LLM-driven task planning via Azure PromptOrchestrator
✅ Chain-of-Thought reasoning with multi-step logic
✅ ROS2/Gazebo integration for robotic simulation
✅ Multi-robot coordination across multiple floors
✅ Real-time telemetry and workflow adaptation
✅ IFC-aligned Digital Twin building simulation
