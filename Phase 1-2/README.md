# Phase 1 & Phase 2 — Azure BIM to Digital Twin & AI Task Planning

## 📌 Project Overview
This folder contains the combined code for **Phase 1** and **Phase 2** of the **Azure BIM to Digital Twin & Robotics Automation Pipeline**.  
The system converts BIM models into **Azure Digital Twins**, stores them in **Azure Blob Storage**, and uses **AI** to generate construction task plans for robotic execution.

---

## 🎯 Objectives

### **Phase 1 — BIM to Digital Twin Conversion**
- Upload IFC/Revit BIM models to **Azure Blob Storage**.
- Parse BIM models into **DTDL-compliant Digital Twin models**.
- Create and populate instances in **Azure Digital Twins (ADT)**.

### **Phase 2 — AI Task Planning & Orchestration**
- Use **Azure OpenAI** with **PromptOrchestrator** to generate step-by-step construction tasks.
- Convert **natural language instructions** into **structured JSON commands** for robotics.
- Provide APIs to retrieve **robot status** and assigned tasks.

---

## ✨ Features
- **BIM Model Ingestion** → IFC/Revit → Blob Storage → Azure Digital Twins  
- **DTDL Model Auto-Upload** to ADT  
- **AI-driven Task Planning** via Azure OpenAI  
- **Robot Status API** for real-time monitoring  
- Fully deployable to **Azure Functions** (Linux, Python 3.10+)

---

## 📂 Folder Structure
phase1-2/
├── deploy_complete_pipeline.sh # Deployment script to Azure
├── function_app.py # Main Azure Functions app (Phase 1 & 2)
├── host.json # Azure Functions host configuration
├── local.settings.json # Local dev settings (DO NOT commit real keys)
├── requirements.txt # Python dependencies
├── test_complete_pipeline.py # Local/remote validation tests
└── README.md # Documentation for Phase 1 & 2

yaml
Copy
Edit

---

## ⚙️ Prerequisites

**Azure Subscription with:**
- Azure Digital Twins
- Azure Blob Storage
- Azure Function App (Linux, Python 3.10+)
- Azure OpenAI

**Tools Required:**
```bash
az login
az account set --subscription "<Your_Subscription_Name>"
npm install -g azure-functions-core-tools@4 --unsafe-perm true
🚀 Deployment
To deploy the Phase 1 & Phase 2 pipeline to Azure Functions:

bash
Copy
Edit
func azure functionapp publish <FunctionAppName> --python
🧪 Testing
Run the test script locally:

bash
Copy
Edit
python3 test_complete_pipeline.py
Logs and telemetry can be viewed in:

mathematica
Copy
Edit
Azure Portal → Function App → Monitor
🔑 Environment Variables
Set the following in Azure Function Configuration:

DIGITAL_TWINS_URL

AZURE_OPENAI_API_KEY

STORAGE_ACCOUNT

STORAGE_KEY

Important: Do not store secrets in local.settings.json when committing to GitHub.

📦 Outputs
After running this phase:

BIM model is uploaded to Azure Blob Storage

Digital Twin model is created in Azure Digital Twins

AI-generated construction task plan is stored in JSON format

Robot status API available for real-time monitoring

