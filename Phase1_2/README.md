Azure BIM → Robot Pipeline (Enhanced) — Phase 1 & Phase 2
End-to-end Azure Functions implementation that:

Converts IFC/Revit BIM into Azure Digital Twins (DTDL) ✅

Orchestrates LLM task planning (Azure OpenAI) and produces ROS2-ready outputs ✅

Adds multi-floor navigation, semantic space targeting, floor-aware task distribution, and secure key handling ✅

Version: 10.0 (Production-ready with all enhancements)

What’s in this phase?
Phase 1 — BIM → Digital Twin
Upload IFC/Revit BIM and parse with enhanced semantic extraction.

Create DTDL models (Building/Floor/Space with CommandRequest) and ingest twins + relationships.

(Optional) Store BIM file in Azure Blob Storage.

Deliverables

Twin graph visible in Azure Digital Twins Explorer

Ingestion via Azure Function (UploadBIM) and DTDL uploader (ModelUploader)

Phase 2 — LLM Task Orchestration
Azure OpenAI (GPT-4o) for plan generation and multi-floor navigation.

Converts LLM output to structured JSON/YAML.

Emits floor-aware ROS2 command bundles.

Deliverables

Prompt orchestration logic (PromptOrchestrator / TaskPlanner)

Sample JSON task plan output + ROS2 command pack (TaskOutputGenerator)

Repo layout (created by the bootstrap script)
pgsql
Copy
Edit
azure-bim-robot-enhanced/
├── function_app.py          # All Phase 1/2 HTTP-triggered functions
├── requirements.txt         # Python deps
├── host.json                # Azure Functions host configuration
├── local.settings.json      # Local dev settings (no secrets)
└── deploy_enhanced.sh       # One-command publish helper
Required Azure resources
Azure Function App (Python)

Azure Digital Twins instance

Azure OpenAI (deployment capable of gpt-4o)

(Optional) Azure Storage (Blob) for BIM file retention

Environment configuration
Set these as Application Settings in your Function App (Azure Portal → Configuration):

Key	Required	Description
DIGITAL_TWINS_URL	✅	Your ADT endpoint (e.g., https://<adtname>.<region>.digitaltwins.azure.net)
AZURE_OPENAI_ENDPOINT	✅	Your Azure OpenAI endpoint (e.g., https://<name>.openai.azure.com/)
AZURE_OPENAI_API_KEY	✅	Do not hardcode. Store as App Setting or in Key Vault.
STORAGE_CONNECTION_STRING	◻️	If you want IFC files automatically uploaded to Blob

Local dev: local.settings.json is provided without any secrets. Export AZURE_OPENAI_API_KEY in your shell for local testing.

Install & run locally
bash
Copy
Edit
# from inside azure-bim-robot-enhanced/
python -m venv .venv
source .venv/bin/activate   # Windows: .venv\Scripts\activate
pip install -r requirements.txt

# Azure Functions Core Tools must be installed
func start
Deploy to Azure
bash
Copy
Edit
# Ensure AZURE_OPENAI_API_KEY is set as an App Setting before production use
bash deploy_enhanced.sh
DTDL models (highlights)
Models are uploaded by ModelUploader and include CommandRequest (corrected) and contains relationships:

dtmi:construction:robot:Building;1

dtmi:construction:robot:Floor;1

dtmi:construction:robot:Space;1

Notable properties/telemetry:

Building: buildingName, buildingType, overallProgress, robotFleetStatus

Floor: floorName, elevation, constructionProgress, contains -> Space

Space: spaceName, spaceType, area, robotTaskZone, navigationWaypoints

HTTP Endpoints
All functions are HTTP-triggered. Routes below assume default host.

1) Upload DTDL models
GET /api/ModelUploader

Uploads the 3 DTDL models to ADT.

Response (200)

json
Copy
Edit
{
  "status": "success",
  "message": "Uploaded 3 ENHANCED robot-ready DTDL models",
  "models": [
    "dtmi:construction:robot:Space;1",
    "dtmi:construction:robot:Floor;1",
    "dtmi:construction:robot:Building;1"
  ]
}
2) Upload & parse BIM (IFC) → create twins
POST /api/UploadBIM?filename=New_Actual_Building.ifc
Body: raw IFC bytes

Stores IFC to Blob (if STORAGE_CONNECTION_STRING set)

Parses IFC with semantic name extraction

Creates Building → Floor → Space twins + contains relationships

Auto-assigns robot task zones and navigation waypoints per space

Curl

bash
Copy
Edit
curl -X POST "http://localhost:7071/api/UploadBIM?filename=Building.ifc" \
  --data-binary @./Building.ifc \
  -H "Content-Type: application/octet-stream"
Response (200)

json
Copy
Edit
{
  "status": "success",
  "building_id": "building-building-ifc",
  "floors_created": 2,
  "rooms_created": 6,
  "building_data": { "...": "..." }
}
3) Get robot fleet status (simulated)
GET /api/RobotStatus?building_id=<id>

Returns a simple fleet snapshot (multi-floor positions).

4) Plan tasks with Azure OpenAI
GET /api/TaskPlanner?building_id=<id>&task_type=construction_sequence

Pulls building/floor context from ADT

Uses Azure OpenAI (GPT-4o) to produce a plan

Returns JSON task plan (falls back to a safe default on parse errors)

Response (200)

json
Copy
Edit
{
  "status": "success",
  "task_plan": { "tasks": [ ... ] }
}
5) Prompt orchestrator (reasoned plan + multi-floor nav)
GET /api/PromptOrchestrator?building_id=<id>&complexity=detailed&min_nav_per_floor=4&require_upper_floor=true

Executes a multi-step reasoning workflow:

Site analysis (uses ADT floor/space names/elevations)

Sequencing across floors

Final compact JSON with ≥N tasks per floor and upper floor coverage

Returns reasoning_steps (text) and final_tasks (JSON).
(Note: We avoid logging private chain-of-thought in production logs.)

6) Convert plan → ROS2 command bundle
POST /api/TaskOutputGenerator?format=json&robot_type=construction_robot
Body: task JSON from TaskPlanner or PromptOrchestrator

Normalizes tasks and outputs floor-aware ROS2 commands (adds at least 2 waypoints per nav task; correct z by floor elevation)

Example request body

json
Copy
Edit
{
  "final_tasks": {
    "tasks": [
      {
        "task_id": "T001",
        "description": "Navigate to reception on floor 0",
        "command_type": "navigation",
        "floor": 0,
        "parameters": {
          "space_name": "reception",
          "target_position": {"x": 4.0, "y": 0.5, "z": 0.5}
        }
      }
    ]
  }
}
Response (200)

json
Copy
Edit
{
  "status": "success",
  "ros2_commands": {
    "format": "ros2_construction_commands",
    "tasks": [
      {
        "task_id": "T001",
        "ros2_commands": [
          { "command_type": "navigation", "topic": "/move_base_simple/goal", "message_type": "geometry_msgs/PoseStamped", "parameters": { "target_position": {"x": 4.0,"y": 0.5,"z": 0.5} } },
          { "command_type": "navigation", "topic": "/move_base_simple/goal", "message_type": "geometry_msgs/PoseStamped", "parameters": { "target_position": {"x": 4.6,"y": 0.9,"z": 0.5} } }
        ]
      }
    ]
  }
}
Key enhancements in this build
Multi-floor navigation

z = elevation + 0.5 heuristic; caches floor elevations from ADT

Per-task minimum nav count; avoids single-point patrols

Semantic space targeting

Robust IFC parsing to derive meaningful names and areas

Space type classification (kitchen/bathroom/office/etc.)

Floor-aware task distribution

Ensures coverage on ground and upper floors

Security

No keys in code; reads AZURE_OPENAI_API_KEY from environment

Recommend Azure Key Vault + Function App managed identity

Blob storage optional and sandboxed container

Testing quickstart
Upload models

bash
Copy
Edit
curl http://localhost:7071/api/ModelUploader
Upload IFC

bash
Copy
Edit
curl -X POST "http://localhost:7071/api/UploadBIM?filename=Building.ifc" \
  --data-binary @./Building.ifc -H "Content-Type: application/octet-stream"
Plan tasks

bash
Copy
Edit
curl "http://localhost:7071/api/PromptOrchestrator?building_id=<YOUR_ID>&min_nav_per_floor=4&require_upper_floor=true"
Generate ROS2 output

bash
Copy
Edit
curl -X POST "http://localhost:7071/api/TaskOutputGenerator?format=json" \
  -H "Content-Type: application/json" \
  -d @final_tasks.json
Known limitations
IFC parsing is regex-based (no full IFC geometry). It extracts names/types/areas heuristically.

Space → floor assignment uses a simple distribution if explicit relationships aren’t present.

Example RobotStatus is simulated. Wire this to your real telemetry in later phases.

Troubleshooting
401/403 with ADT: Ensure the Function App’s identity has Azure Digital Twins Data Owner/Reader on the ADT resource.

OpenAI errors: Verify AZURE_OPENAI_ENDPOINT and AZURE_OPENAI_API_KEY. Confirm a deployed model named for gpt-4o.

Blob upload fails: Leave STORAGE_CONNECTION_STRING empty to skip storage, or provide a valid connection string.

License & Credits
Built for the Azure-based BIM → Robot Simulation Pipeline PoC.

Uses Azure SDKs, Azure Functions, Azure Digital Twins, Azure OpenAI, and ROS2 message formats.
