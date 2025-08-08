# Phase 1 & Phase 2 — Azure BIM to Digital Twin & AI Task Planning

## 📌 Project Overview
This folder contains the combined code for **Phase 1** and **Phase 2** of the **Azure BIM to Digital Twin & Robotics Automation Pipeline**.  
The system converts BIM models into Azure Digital Twins, stores them in Azure Blob Storage, and uses AI to generate construction task plans for robotic execution.

---

## 🎯 Objectives

### **Phase 1 — BIM to Digital Twin Conversion**
- Upload IFC/Revit BIM models to Azure Blob Storage.
- Parse BIM models into DTDL-compliant Digital Twin models.
- Create and populate instances in Azure Digital Twins (ADT).

### **Phase 2 — AI Task Planning & Orchestration**
- Use Azure OpenAI with PromptOrchestrator to generate step-by-step construction tasks.
- Convert natural language instructions into structured JSON commands for robotics.
- Provide APIs to retrieve robot status and assigned tasks.

---

## ✨ Features
- **BIM Model Ingestion** → IFC/Revit → Blob Storage → Azure Digital Twins
- **DTDL Model Auto-Upload** to ADT
- **AI-driven Task Planning** via Azure OpenAI
- **Robot Status API** for real-time monitoring
- Fully deployable to **Azure Functions** (Linux, Python 3.10+)

---

## 📂 Folder Structure
