# Phase 3 – Robotic Task Simulation with ROS2 & Gazebo

## 📌 Overview
This folder contains the **Phase 3 deliverables** for the **Azure-Based BIM to Robot Simulation Pipeline**.  

Phase 3 demonstrates the complete integration of **Azure PromptOrchestrator** (from Phase 2) with **ROS2** and **Gazebo** to enable **LLM-driven multi-step construction task planning** using **Chain-of-Thought reasoning**. The system simulates a **2-robot construction workflow** inside an **IFC-aligned Digital Twin building** with real-time task execution and telemetry feedback.

---

## 📋 Deliverables

### **Deliverable 1 – ROS2 Node to Simulate Task**
**File:** `phase3_prompt_orchestrator_executor.py`  
**Size:** 25 KB  

**Description:**  
Core ROS2 node that bridges Azure cloud intelligence with robotic simulation. It fetches Chain-of-Thought reasoning from Azure PromptOrchestrator and translates it into executable robot commands.

**Key Features:**
- Connects to Azure PromptOrchestrator endpoint every 45 seconds
- Processes 3-step Chain-of-Thought reasoning (Site Analysis → Task Sequencing → Execution)
- Controls 2 specialized construction robots (ground floor & upper floor)
- Publishes ROS2 navigation goals (`/move_base_simple/goal`)
- Generates real-time telemetry in Azure-compatible JSON format
- Tracks reasoning history and task completion

**Key Functions:**
```python
fetch_prompt_orchestration()     # Fetches LLM reasoning from Azure
process_reasoning_steps()         # Converts reasoning to structured tasks
execute_llm_driven_movements()    # Moves robots based on LLM tasks
publish_digital_twin_status()     # Sends telemetry to Azure

Deliverable 2 – Simulated Robot Workflow
File: enhanced_workflow_simulator.py
Size: 22 KB
Description:
Advanced workflow simulator that manages the entire construction process through 6 adaptive phases, responding dynamically to LLM reasoning updates.
Key Features:

6-phase construction workflow with reasoning-based adaptation
Subscribes to /llm/reasoning_steps for real-time updates
Coordinates multi-robot movements across floors
Implements safety verification and quality checkpoints
Generates workflow completion reports

Construction Phases:

Site Analysis – Complete building survey with accessibility checks
Foundation Preparation – Excavation and foundation marking
Structural Assembly – Column, beam, and wall construction
Upper Floor Construction – Second floor parallel execution
Systems Integration – MEP (Mechanical, Electrical, Plumbing) installation
Quality Finishing – Final inspection and certification


📂 Supporting Files
Configuration Files

azure_config_advanced.json (3 KB) – Azure endpoints and building specifications
chain_of_thought_history.json (189 KB) – Logged LLM reasoning steps
prompt_orchestrator_telemetry.json (2 KB) – Real-time system telemetry
phase3_promptorchestrator_compliance.json (1 KB) – Compliance verification results

Simulation Files

phase3_ifc_digital_twin.sdf (17 KB) – Gazebo world with IFC-aligned building
phase3_advanced_task_executor.py (15 KB) – Alternative task executor
verify_phase3_promptorchestrator.py (9 KB) – Automated compliance checker

Launch Scripts

launch_phase3_ifc_aligned.sh (2 KB) – Basic launcher
launch_phase3_promptorchestrator.sh (5 KB) – Full system launcher


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
Quick Start
bash# 1. Clone or download all Phase 3 files to a directory
mkdir ~/phase3_ros_gazebo
cd ~/phase3_ros_gazebo

# 2. Copy all deliverables
# Copy all files from the Phase3_Deliverables folder

# 3. Launch the system
./launch_phase3_promptorchestrator.sh

🚀 Running the System
Method 1: Using Launch Script (Recommended)
bash./launch_phase3_promptorchestrator.sh
Method 2: Manual Component Launch
bash# Terminal 1: Start Gazebo
gz sim phase3_ifc_digital_twin.sdf

# Terminal 2: Start ROS2 executor (Deliverable 1)
python3 phase3_prompt_orchestrator_executor.py

# Terminal 3: Start workflow simulator (Deliverable 2)
python3 enhanced_workflow_simulator.py
Method 3: Minimal Test Run
bash# Source ROS2
source /opt/ros/humble/setup.bash

# Kill existing processes
pkill -f gazebo; pkill -f gz; pkill -f ros2

# Start all components
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
Generated Output Files
FileDescriptionUpdate Frequencyprompt_orchestrator_telemetry.jsonRobot positions, task statusEvery 10 secondschain_of_thought_history.jsonLLM reasoning stepsEvery 45 secondsreasoning_workflow_*.jsonCompleted workflow reportsPer workflowphase3_promptorchestrator_completion.jsonFinal system reportOn completion

🏢 Digital Twin Building Specifications
Structure: Mixed-use Residential/Commercial
Total Area: 73 m²
Floors: 2 (Ground + Upper)
Total Spaces: 5
Ground Floor (Elevation: 0.0m)

BEDROOM (01) – 15 m² residential space
BEDROOM (02) – 15 m² residential space
KITCHEN (102) – 12 m² utility space
BATHROOM (103) – 6 m² utility space

Upper Floor (Elevation: 3.5m)

LOUNGE (104) – 25 m² commercial space

Construction Robots

azure_construction_robot_01 – Ground floor specialist (cyan)
azure_construction_robot_02 – Upper floor specialist (pink)


🔗 Integration with Other Phases
Input from Phase 2

PromptOrchestrator endpoint: /api/PromptOrchestrator
Chain-of-Thought reasoning with 3 steps
Structured JSON task plans with floor assignments
Safety protocols and dependencies

Output to Phase 4

Real-time telemetry data
Task completion status
Workflow progress metrics
System performance logs


👥 For Other Users
If you want to run Phase 3 on your own setup, follow these steps.
1️⃣ Prerequisites
System Requirements

ROS2 Humble + Gazebo Garden installed
Python 3.8+ environment
Ubuntu 22.04 (recommended) or compatible OS
GPU support recommended for Gazebo visualization

Azure Resources (Optional but recommended)

Azure OpenAI – For task reasoning via PromptOrchestrator
Azure Digital Twins – For telemetry integration
Azure Function App – Hosting Phase 2 endpoints

2️⃣ Environment Setup
Install ROS2 Dependencies
bash# Add ROS2 repository
sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Install ROS2 Humble
sudo apt update
sudo apt install ros-humble-desktop

# Source ROS2
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
Install Python Dependencies
bash# Create virtual environment (optional but recommended)
python3 -m venv phase3_env
source phase3_env/bin/activate

# Install required packages
pip install requests rclpy geometry-msgs std-msgs pyyaml
3️⃣ Configuration
Edit Azure Configuration
Update azure_config_advanced.json with your Azure endpoints:
json{
  "base_url": "https://YOUR-FUNCTION-APP.azurewebsites.net",
  "function_code": "YOUR-FUNCTION-CODE",
  "building_id": "building-your-ifc-file",
  "endpoints": {
    "prompt_orchestrator": "/api/PromptOrchestrator",
    "task_output_generator": "/api/TaskOutputGenerator"
  },
  "orchestration_settings": {
    "complexity": "detailed",
    "enable_chain_of_thought": true,
    "reasoning_depth": 3
  },
  "world_settings": {
    "world_name": "phase3_ifc_digital_twin",
    "building_area": 73,
    "robot1_start": {"x": -3.5, "y": -2.0, "z": 0.5},
    "robot2_start": {"x": 4.0, "y": 1.5, "z": 4.7}
  }
}
Local Testing Without Azure
For testing without Azure connection, set mock mode in the executor:
python# In phase3_prompt_orchestrator_executor.py, line 45:
MOCK_MODE = True  # Set to True for local testing without Azure
4️⃣ Building Your Own World
If you want to use your own building model:

Create SDF file from your IFC model
Update robot spawn positions in the SDF
Modify space coordinates in azure_config_advanced.json
Adjust floor elevations based on your building

5️⃣ Launch Sequence
Option A: Full System Launch
bash# Navigate to Phase 3 directory
cd ~/phase3_ros_gazebo

# Make scripts executable
chmod +x launch_phase3_promptorchestrator.sh

# Launch everything
./launch_phase3_promptorchestrator.sh
Option B: Component-by-Component
bash# Terminal 1: Launch Gazebo with your world
gz sim phase3_ifc_digital_twin.sdf

# Terminal 2: Start ROS2 executor
python3 phase3_prompt_orchestrator_executor.py

# Terminal 3: Start workflow simulator
python3 enhanced_workflow_simulator.py
6️⃣ Verification & Testing
Check System Status
bash# Verify ROS2 topics are active
ros2 topic list

# Should show:
# /move_base_simple/goal
# /excavator/cmd_vel
# /robot/level
# /robot/install
# /robot/finish
# /digital_twin/status
# /llm/reasoning_steps
# /robot_workflow_status
Run Compliance Check
bashpython3 verify_phase3_promptorchestrator.py
7️⃣ Troubleshooting for New Users
IssueSolution"No module named 'rclpy'"Install ROS2: sudo apt install python3-rclpy"gz: command not found"Install Gazebo: sudo apt install gz-garden"Connection refused to Azure"Check firewall and Azure Function URL"Robots not visible in Gazebo"Check SDF file path and robot spawn coordinates"No reasoning steps received"Verify PromptOrchestrator endpoint in Phase 2
8️⃣ Customization Options
Modify Robot Behavior
Edit enhanced_workflow_simulator.py:
python# Line 89: Adjust workflow timing
self.workflow_timer = self.create_timer(10.0, self.advance_workflow)  # Change interval

# Line 156: Modify robot speed
cmd.linear.x = 0.5  # Increase speed
Change Building Layout
Edit phase3_ifc_digital_twin.sdf:
xml<!-- Add new room -->
<model name="new_room">
  <pose>x y z 0 0 0</pose>
  <!-- Room geometry -->
</model>

✅ Verification & Testing
Run Compliance Check
bashpython3 verify_phase3_promptorchestrator.py
Expected Output
✅ Deliverable 1 (ROS2 + PromptOrchestrator): PASS
✅ Deliverable 2 (Enhanced Workflow): PASS
🧠 Chain-of-Thought Integration: PASS
⚙️ System Requirements: PASS
🎯 OVERALL PHASE 3 COMPLIANCE: ✅ PASS

🛠️ Troubleshooting
Common Issues
IssueSolution"Azure configuration not found"Ensure azure_config_advanced.json is in the same directory"ROS2 command not found"Source ROS2: source /opt/ros/humble/setup.bash"Gazebo fails to start"Check GPU drivers: gz sim --version"No LLM tasks received"Verify Azure endpoint URL and function code in config"Robots not moving"Check Gazebo is running and robots are spawned
Debug Commands
bash# Check ROS2 topics
ros2 topic list

# Monitor navigation goals
ros2 topic echo /move_base_simple/goal

# View robot status
ros2 topic echo /digital_twin/status

📈 Performance Metrics
System Performance:

LLM reasoning fetch: Every 45 seconds
Robot movement update: Every 4 seconds
Workflow step advancement: Every 10 seconds
Telemetry publication: Every 10 seconds

Typical Execution Times:

Full 6-phase workflow: ~5-10 minutes
Single workflow phase: 30-90 seconds
LLM reasoning processing: 10-20 seconds
Robot navigation task: 2-4 seconds


📝 License & Attribution
This Phase 3 implementation is part of the Azure-Based BIM to Robot Simulation Pipeline proof-of-concept.
Technologies Used:

ROS2 Humble
Gazebo Garden
Azure PromptOrchestrator
Azure Digital Twins
Python 3.8+


🎯 Summary
Phase 3 successfully demonstrates:

✅ LLM-driven task planning via Azure PromptOrchestrator
✅ Chain-of-Thought reasoning with multi-step logic
✅ ROS2/Gazebo integration for robotic simulation
✅ Multi-robot coordination across multiple floors
✅ Real-time telemetry and workflow adaptation
✅ IFC-aligned Digital Twin building simulation

The deliverables provide a complete, working system that bridges cloud-based AI reasoning with local robotic simulation, ready for Phase 4 automation and orchestration.
