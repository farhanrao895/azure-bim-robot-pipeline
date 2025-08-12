# 🏗️ Azure BIM → Robot Pipeline (Enhanced)
## Phase 1 & Phase 2 Implementation

[![Azure Functions](https://img.shields.io/badge/Azure-Functions-0078D4?logo=azure-functions)](https://azure.microsoft.com/services/functions/)
[![Python](https://img.shields.io/badge/Python-3.10+-3776AB?logo=python)](https://www.python.org/)
[![ROS2](https://img.shields.io/badge/ROS2-Ready-22314E?logo=ros)](https://www.ros.org/)
[![Version](https://img.shields.io/badge/Version-10.0-green)](https://github.com)

> **Production-ready implementation** that transforms BIM models into Azure Digital Twins and orchestrates AI-driven construction task planning with multi-floor navigation support.

---

## 🎯 What This Does

- **Converts** IFC/Revit BIM models → Azure Digital Twins (DTDL) ✅
- **Orchestrates** LLM task planning via Azure OpenAI (GPT-4o) ✅  
- **Generates** ROS2-ready command sequences for robot control ✅
- **Supports** multi-floor navigation with semantic space targeting ✅

---

## 📋 Phase Overview

### **Phase 1: BIM → Digital Twin**
Transform building information models into cloud-native digital representations.

| Feature | Description |
|---------|-------------|
| **IFC Parser** | Enhanced semantic extraction (room names, types, areas) |
| **DTDL Models** | Building/Floor/Space hierarchy with `CommandRequest` |
| **Twin Creation** | Automated ingestion with relationships |
| **Blob Storage** | Optional BIM file retention |

**Key Functions:**
- `ModelUploader` - Deploys DTDL schemas to ADT
- `UploadBIM` - Processes IFC files and creates twins
- `RobotStatus` - Monitors fleet status

### **Phase 2: LLM Task Orchestration**
Leverage AI for intelligent construction planning and task distribution.

| Feature | Description |
|---------|-------------|
| **Azure OpenAI** | GPT-4o for Chain-of-Thought reasoning |
| **Multi-Floor Logic** | Floor-aware task distribution |
| **Structured Output** | JSON/YAML task plans |
| **ROS2 Commands** | Navigation goals with proper z-coordinates |

**Key Functions:**
- `TaskPlanner` - Basic AI task generation
- `PromptOrchestrator` - Advanced multi-step reasoning
- `TaskOutputGenerator` - ROS2 command conversion

---

## 🚀 Quick Start

### **Prerequisites**
```bash
# Required tools
- Python 3.10+
- Azure CLI
- Azure Functions Core Tools

# Install dependencies
pip install -r requirements.txt
Local Development
bash# Clone repository
git clone <repository-url>
cd azure-bim-robot-enhanced

# Set up environment
python -m venv .venv
source .venv/bin/activate  # Windows: .venv\Scripts\activate

# Configure API key (do not commit!)
export AZURE_OPENAI_API_KEY="your-key-here"

# Start Functions locally
func start
Deploy to Azure
bash# One-command deployment
bash deploy_enhanced.sh

# Or manual deployment
func azure functionapp publish <your-function-app-name> --python

⚙️ Configuration
Required Azure Resources
ResourcePurposeAzure Function AppHosts all HTTP-triggered functionsAzure Digital TwinsStores building digital representationAzure OpenAIProvides GPT-4o for task planningAzure Storage (Optional)Stores uploaded BIM files
Environment Variables
Configure in Azure Portal → Function App → Configuration:
VariableRequiredDescriptionDIGITAL_TWINS_URL✅ADT endpoint: https://<name>.<region>.digitaltwins.azure.netAZURE_OPENAI_ENDPOINT✅OpenAI endpoint: https://<name>.openai.azure.com/AZURE_OPENAI_API_KEY✅Store in Key Vault or App Settings (never in code)STORAGE_CONNECTION_STRING⭕For BIM file storage in Blob

📡 API Endpoints
1. Upload DTDL Models
httpGET /api/ModelUploader
Deploys robot-ready DTDL schemas to Azure Digital Twins.
2. Process BIM File
httpPOST /api/UploadBIM?filename=Building.ifc
Content-Type: application/octet-stream
Body: <IFC file bytes>
Parses IFC and creates digital twin graph.
3. Get Robot Status
httpGET /api/RobotStatus?building_id=<building-id>
Returns current fleet status and positions.
4. Generate AI Task Plan
httpGET /api/TaskPlanner?building_id=<id>&task_type=construction_sequence
Simple task planning with Azure OpenAI.
5. Chain-of-Thought Planning
httpGET /api/PromptOrchestrator?building_id=<id>&complexity=detailed&min_nav_per_floor=4&require_upper_floor=true
Advanced multi-step reasoning with floor distribution.
6. Convert to ROS2 Commands
httpPOST /api/TaskOutputGenerator?format=json&robot_type=construction_robot
Content-Type: application/json
Body: <task plan JSON>
Transforms AI plans into executable ROS2 commands.

📂 Project Structure
azure-bim-robot-enhanced/
│
├── 📄 function_app.py          # All Phase 1/2 Azure Functions
├── 📄 requirements.txt         # Python dependencies
├── 📄 host.json               # Function runtime config
├── 📄 local.settings.json     # Local dev settings (no secrets)
├── 📄 deploy_enhanced.sh      # Deployment script
└── 📄 README.md              # This file

🏗️ DTDL Models
The system uses three hierarchical models:
mermaidgraph TD
    A[Building] -->|contains| B[Floor]
    B -->|contains| C[Space]
Key Properties:

Building: buildingName, buildingType, totalArea, robotDeploymentZones
Floor: floorName, elevation, robotAccessible, constructionPhase
Space: spaceName, spaceType, area, navigationWaypoints, robotTaskZone


✨ Key Enhancements
EnhancementDescription🏢 Multi-Floor NavigationProper z-coordinate calculation based on floor elevation🎯 Semantic Space TargetingNavigate to "Executive Office" not just coordinates⚖️ Floor-Aware DistributionBalanced task assignment across floors🔒 Secure Key HandlingAPI keys in environment variables, not code🧠 Chain-of-ThoughtMulti-step reasoning for complex planning

🧪 Testing Examples
Complete Pipeline Test
bash# 1. Upload DTDL models
curl http://localhost:7071/api/ModelUploader

# 2. Upload IFC file
curl -X POST "http://localhost:7071/api/UploadBIM?filename=Building.ifc" \
  --data-binary @./Building.ifc \
  -H "Content-Type: application/octet-stream"

# 3. Generate task plan with reasoning
curl "http://localhost:7071/api/PromptOrchestrator?building_id=building-test&complexity=detailed"

# 4. Convert to ROS2 commands
curl -X POST "http://localhost:7071/api/TaskOutputGenerator?format=json" \
  -H "Content-Type: application/json" \
  -d '{"tasks": [...]}'

👥 For Other Users
Step-by-Step Setup
1️⃣ Azure Resources
Create these in Azure Portal:

Azure Digital Twins instance
Azure OpenAI resource (with GPT-4o deployment)
Azure Function App (Python 3.10 runtime)
Azure Storage Account (optional)

2️⃣ Role Assignments
Grant your Function App's managed identity:

Azure Digital Twins Data Owner on your ADT instance
Cognitive Services OpenAI User on your OpenAI resource

3️⃣ Configuration
In Azure Portal → Function App → Configuration → Application Settings:
json{
  "DIGITAL_TWINS_URL": "https://your-adt.region.digitaltwins.azure.net",
  "AZURE_OPENAI_ENDPOINT": "https://your-openai.openai.azure.com/",
  "AZURE_OPENAI_API_KEY": "store-in-keyvault-reference"
}
4️⃣ Deploy Code
bash# Clone this repository
git clone <repo-url>
cd azure-bim-robot-enhanced

# Deploy to your Function App
func azure functionapp publish YOUR-FUNCTION-APP-NAME --python
5️⃣ Verify Deployment
Test each endpoint using the examples above, replacing localhost:7071 with your Function App URL.

📌 Code Availability
This repository contains full runnable codebases:

Phase 1–2 complete code → Located in the Phase 1-2 folder (Complete_Phase_1_2_Code.py file) — Creates all components for those
phases.

⚠️ Known Limitations

IFC Parsing: Regex-based extraction (not full geometry processing)
Floor Mapping: Heuristic when explicit relationships missing
Robot Status: Currently simulated (replace with actual telemetry)
Scale: Optimized for buildings < 100 spaces


📝 License
This implementation is part of the Azure-Based BIM to Robot Simulation Pipeline proof-of-concept.

🤝 Support
For issues or questions:

Check the Known Limitations section
Review Azure Function logs in Application Insights
Ensure all environment variables are correctly set


