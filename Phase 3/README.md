# 🤖 Phase 3 – Robotic Task Simulation with ROS2 & Gazebo

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

