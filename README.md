# Azure BIM to Robot Pipeline – End-to-End Orchestration

## Overview
This repository contains the complete multi-phase implementation of the **Azure BIM to Digital Twin & Robotic Task Simulation Pipeline**.  
It demonstrates an end-to-end workflow starting from a Building Information Model (BIM) in IFC/Revit format, converting it to an **Azure Digital Twin**, using an **LLM (Azure OpenAI)** for **Chain-of-Thought** construction task planning, and executing the tasks in **ROS2 & Gazebo** robotic simulations.  
It also includes **IoT telemetry dashboards** for real-time monitoring of robotic activities.

The pipeline is modular, with each phase building on the previous one.

---

## 📂 Repository Structure

| Phase | Folder | Description |
|------:|:------:|-------------|
| **Phase 1-2** | [Phase 1-2](./Phase%201-2) | **BIM → Digital Twin Conversion** (Phase 1) and **LLM Task Orchestration** (Phase 2). Converts BIM models into Azure Digital Twins, stores them in Azure Blob Storage, and uses Azure OpenAI to generate structured task plans. |
| **Phase 3** | [Phase 3](./Phase%203) | **Robotic Task Simulation.** Integrates Azure PromptOrchestrator with ROS2 & Gazebo to simulate multi-robot construction workflows using LLM-driven multi-step reasoning. |
| **Phase 4** | [Phase 4](./Phase%204) | **Automation & Orchestration.** Integrates IoT telemetry dashboards for monitoring robotic activity in real time. |
| **Phase 5** | *(This repo)* | **Version Control & DevOps.** Full GitHub repository with documentation, deployment scripts, and test data for all completed phases. |

---

## 🚀 Pipeline Workflow

### Phase 1 – BIM to Digital Twin Conversion
- Convert IFC/Revit BIM models into DTDL-compliant Azure Digital Twin models.  
- Store models in Azure Blob Storage.  
- Ingest models into Azure Digital Twins Explorer.

### Phase 2 – LLM Task Orchestration
- Use Azure OpenAI with **Chain-of-Thought** prompting to generate detailed, structured task plans.  
- Output JSON-formatted plans compatible with robotic control systems.

### Phase 3 – Robotic Task Simulation
- Simulate multi-robot construction workflows in **Gazebo**.  
- Fetch reasoning-driven task plans from **Azure PromptOrchestrator**.  
- Execute tasks in real time with telemetry feedback to Azure.

### Phase 4 – Automation & Orchestration
- Integrate IoT telemetry dashboards for monitoring robotic activity in real time.

### Phase 5 – Version Control & DevOps
- Maintain complete infrastructure and application code in GitHub.  
- Provide clear documentation, run instructions, and test data.

---

## 📜 How to Run Each Phase

- **Phase 1-2:** See [Phase 1-2/README.md](./Phase%201-2/README.md) for detailed setup and execution instructions.  
- **Phase 3:** See [Phase 3/README.md](./Phase%203/README.md) for complete instructions on running the ROS2 & Gazebo simulation with PromptOrchestrator.  
- **Phase 4:** See [Phase 4/README.md](./Phase%204/README.md) for instructions on running the IoT telemetry dashboard.

📌 Code Availability
This repository contains two full runnable codebases:

Phase 1–2 complete code → Located in the Phase 1-2 folder (.py file) — Creates all components for those phases.

Phase 3 complete code → Located in the Phase 3 folder (.py file) — Creates and runs the full simulation.

Each can be run independently in their respective folders.



---

## 🔧 Prerequisites

- Azure Subscription with:
  - **Azure Digital Twins**
  - **Azure OpenAI**
  - **Azure Functions**
  - **Azure Blob Storage**
- **ROS2 Humble**
- **Gazebo Garden**
- **Python 3.8+**
- (Optional for Phase 4) **IoT Hub / Dashboard Environment** (Power BI, Grafana, or similar)

---

## 📊 Deliverables Summary

**Completed**
- ✅ BIM → Digital Twin Conversion (**Phase 1**)
- ✅ LLM Task Planning (**Phase 2**)
- ✅ Multi-Robot Simulation with Reasoning (**Phase 3**)
- ✅ IoT Telemetry Dashboard Integration (**Phase 4**)


---

## 📌 Notes
- Phase 5 focuses on **packaging the completed work** into a **version-controlled, documented repository**.  
- Phase 4 is limited to **dashboard integration** for real-time monitoring in this implementation.  

