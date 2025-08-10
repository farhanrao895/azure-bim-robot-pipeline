# Azure BIM → Robot Pipeline (Enhanced) — Phase 1 & Phase 2

End-to-end **Azure Functions** implementation that:
- Converts **IFC/Revit BIM** into **Azure Digital Twins** (DTDL) ✅
- Orchestrates **LLM task planning** (Azure OpenAI) and produces **ROS2-ready** outputs ✅
- Adds **multi-floor navigation**, **semantic space targeting**, **floor-aware task distribution**, and **secure key handling** ✅

> **Version:** 10.0 — Production-ready with all enhancements.

---

## 📌 Phase Overview

### **Phase 1 — BIM → Digital Twin**
- Upload IFC/Revit BIM and parse with **enhanced semantic extraction**.
- Create **DTDL models** (Building/Floor/Space with `CommandRequest`) and ingest **twins + relationships**.
- (Optional) Store BIM file in **Azure Blob Storage**.

**Deliverables:**
- Twin graph in **Azure Digital Twins Explorer**.
- Ingestion via **Azure Function** (`UploadBIM`) and DTDL uploader (`ModelUploader`).

---

### **Phase 2 — LLM Task Orchestration**
- **Azure OpenAI (GPT-4o)** for plan generation and **multi-floor navigation**.
- Converts LLM output to **structured JSON/YAML**.
- Emits **floor-aware ROS2 command bundles**.

**Deliverables:**
- **Prompt orchestration logic** (`PromptOrchestrator` / `TaskPlanner`).
- **Sample JSON task plan output** + **ROS2 command pack** (`TaskOutputGenerator`).

---

## 📂 Repository Structure

```plaintext
azure-bim-robot-enhanced/
├── function_app.py          # All Phase 1/2 HTTP-triggered functions
├── requirements.txt         # Python dependencies
├── host.json                # Azure Functions host configuration
├── local.settings.json      # Local dev settings (no secrets)
└── deploy_enhanced.sh       # One-command publish helper
🔧 Required Azure Resources
Azure Function App (Python)

Azure Digital Twins instance

Azure OpenAI (deployment capable of gpt-4o)

(Optional) Azure Storage (Blob) for BIM file retention

⚙ Environment Configuration
Set these as Application Settings in your Function App (Azure Portal → Configuration):

Key	Required	Description
DIGITAL_TWINS_URL	✅	Your ADT endpoint (e.g., https://<adtname>.<region>.digitaltwins.azure.net)
AZURE_OPENAI_ENDPOINT	✅	Your Azure OpenAI endpoint (e.g., https://<name>.openai.azure.com/)
AZURE_OPENAI_API_KEY	✅	Do not hardcode. Store in App Settings or Key Vault.
STORAGE_CONNECTION_STRING	◻️	If you want IFC files uploaded to Blob

Local dev: local.settings.json is provided without secrets.
Export AZURE_OPENAI_API_KEY in your shell for local testing.

💻 Install & Run Locally
bash
Copy
Edit
# From inside azure-bim-robot-enhanced/
python -m venv .venv
source .venv/bin/activate   # Windows: .venv\Scripts\activate
pip install -r requirements.txt

# Azure Functions Core Tools must be installed
func start
🚀 Deploy to Azure
bash
Copy
Edit
# Ensure AZURE_OPENAI_API_KEY is set as an App Setting before production
bash deploy_enhanced.sh
🏗 DTDL Models
Models uploaded by ModelUploader include CommandRequest and contains relationships:

dtmi:construction:robot:Building;1

dtmi:construction:robot:Floor;1

dtmi:construction:robot:Space;1

Properties & Telemetry Highlights:

Building: buildingName, buildingType, overallProgress, robotFleetStatus

Floor: floorName, elevation, constructionProgress

Space: spaceName, spaceType, area, robotTaskZone, navigationWaypoints

🌐 API Endpoints
1) Upload DTDL Models
GET /api/ModelUploader

Uploads the 3 enhanced robot-ready DTDL models to ADT.

2) Upload & Parse BIM (IFC) → Create Twins
POST /api/UploadBIM?filename=New_Actual_Building.ifc
Body: IFC file bytes.

3) Get Robot Fleet Status
GET /api/RobotStatus?building_id=<id>

4) Plan Tasks with Azure OpenAI
GET /api/TaskPlanner?building_id=<id>&task_type=construction_sequence

5) Prompt Orchestrator
GET /api/PromptOrchestrator?...
Multi-step reasoning → floor-aware plans.

6) Convert Plan → ROS2 Commands
POST /api/TaskOutputGenerator?format=json&robot_type=construction_robot

✨ Key Enhancements
Multi-floor navigation with elevation-aware waypoints.

Semantic space targeting from IFC data.

Balanced floor coverage in task distribution.

Secure key handling via Azure Key Vault & Function App settings.

🧪 Testing Quickstart
bash
Copy
Edit
# 1) Upload models
curl http://localhost:7071/api/ModelUploader

# 2) Upload IFC
curl -X POST "http://localhost:7071/api/UploadBIM?filename=Building.ifc" \
  --data-binary @./Building.ifc -H "Content-Type: application/octet-stream"

# 3) Plan tasks
curl "http://localhost:7071/api/PromptOrchestrator?building_id=<ID>"

# 4) Generate ROS2 output
curl -X POST "http://localhost:7071/api/TaskOutputGenerator?format=json" \
  -H "Content-Type: application/json" \
  -d @final_tasks.json
⚠ Known Limitations
IFC parsing is regex-based, not full geometry parsing.

Floor/space mapping is heuristic when explicit relationships are missing.

RobotStatus is simulated — replace with actual telemetry.
