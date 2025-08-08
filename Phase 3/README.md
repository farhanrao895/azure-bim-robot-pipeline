
Phase 3 – PromptOrchestrator Integration for ROS2 & Gazebo
Overview
This repository contains the Phase 3 deliverables for the BIM-to-Digital Twin robotic simulation system.
In this phase, we integrate Azure PromptOrchestrator with ROS2 and Gazebo to enable LLM-driven multi-step construction task planning using Chain-of-Thought reasoning.

The system simulates a 2-robot construction workflow inside an IFC-aligned Digital Twin building in Gazebo.
Tasks are fetched from Azure PromptOrchestrator, processed into ROS2 commands, and executed with real-time telemetry and reasoning context.

Deliverables
📋 Deliverable 1 – ROS2 Node to Simulate Task
File: phase3_prompt_orchestrator_executor.py
Features:

Connects to Azure PromptOrchestrator for task planning

Retrieves Chain-of-Thought reasoning from Azure OpenAI LLM

Parses multi-step reasoning (Site Analysis → Task Sequencing → Execution)

Controls two construction robots in Gazebo simulation

Publishes telemetry in Azure-compatible JSON format

Key Functions:

fetch_prompt_orchestration() – Fetches LLM reasoning from Azure

process_reasoning_steps() – Processes reasoning steps into structured form

execute_llm_driven_movements() – Moves robots based on LLM tasks

publish_digital_twin_status() – Sends status & telemetry updates

📋 Deliverable 2 – Simulated Robot Workflow
File: enhanced_workflow_simulator.py
Features:

Simulates a 6-phase construction workflow

Adapts workflow dynamically based on LLM reasoning

Coordinates robot movement for construction tasks

Implements safety and quality checkpoints

Tracks progress through all construction phases

Construction Phases Simulated:

Site Analysis

Foundation Preparation

Structural Assembly

Upper Floor Construction

Systems Integration

Quality Finishing

Supporting Files
These files are not deliverables, but are required to run the system:

phase3_ifc_digital_twin.sdf

Gazebo world file defining the 3D building model (2 floors, 5 rooms)

Includes physics, lighting, and robot definitions

azure_config_advanced.json

Azure Function configuration file

Contains PromptOrchestrator endpoint URLs, building specifications, and robot start positions

verify_phase3_promptorchestrator.py

Automated compliance checker

Validates deliverables, reasoning integration, and system requirements

System Architecture
Main Components:

Azure PromptOrchestrator – Generates detailed reasoning-based task plans

ROS2 Node – Executes tasks in the Gazebo simulation

Gazebo Digital Twin – IFC-aligned simulation environment

Enhanced Workflow Simulator – Manages adaptive construction workflows

Key Features:

Multi-step LLM reasoning with context tracking

Real-time robot task execution

Adaptive workflows with safety & quality control

Structured JSON output for Azure telemetry

How to Run
Prerequisites
ROS2 Humble installed (/opt/ros/humble/setup.bash)

Gazebo Garden installed (gz sim)

Python 3.8+ with required dependencies:

bash
Copy
Edit
pip3 install requests rclpy geometry-msgs std-msgs
Run Commands
bash
Copy
Edit
# Source ROS2
source /opt/ros/humble/setup.bash

# Kill any existing processes
pkill -f gazebo
pkill -f gz
pkill -f prompt
pkill -f ros2

# Start all components
gz sim phase3_ifc_digital_twin.sdf &
sleep 10
python3 phase3_prompt_orchestrator_executor.py &
sleep 3
python3 enhanced_workflow_simulator.py &

# Keep terminal alive
echo "System running. Press Ctrl+C to stop all processes"
wait
Monitoring Outputs
While the system runs, you can monitor key files:

prompt_orchestrator_telemetry.json – Real-time system status

chain_of_thought_history.json – Logged reasoning steps from LLM

reasoning_workflow_*.json – Completed workflow summaries

phase3_promptorchestrator_completion.json – Final Phase 3 report

Summary
✅ Deliverable 1 – phase3_prompt_orchestrator_executor.py – ROS2 node that integrates Azure PromptOrchestrator reasoning into robot control
✅ Deliverable 2 – enhanced_workflow_simulator.py – Adaptive multi-phase construction workflow simulator

This Phase 3 implementation meets all requirements for LLM-driven robotic construction simulation with Chain-of-Thought reasoning, multi-robot coordination, and IFC-aligned Digital Twin execution
