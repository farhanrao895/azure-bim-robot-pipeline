#!/bin/bash
# Complete Pipeline Deployment Commands

echo "🚀 Complete Azure BIM-Robot Pipeline Deployment"
echo "==============================================="
echo "✅ Enhanced Phase 1 + Working Phase 2"
echo "✅ All 6 Functions Ready for Deployment"
echo ""

# Check Azure CLI login
echo "🔐 Checking Azure CLI login..."
if ! az account show > /dev/null 2>&1; then
    echo "Please login to Azure CLI:"
    az login
fi

# Check Azure Functions Core Tools
echo "📦 Checking Azure Functions Core Tools..."
if ! command -v func &> /dev/null; then
    echo "⚠️ Azure Functions Core Tools not found. Installing..."
    if [[ "$OSTYPE" == "linux-gnu"* ]]; then
        # Ubuntu/Debian
        curl https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > microsoft.gpg
        sudo mv microsoft.gpg /etc/apt/trusted.gpg.d/microsoft.gpg
        sudo sh -c 'echo "deb [arch=amd64] https://packages.microsoft.com/repos/microsoft-ubuntu-$(lsb_release -cs)-prod $(lsb_release -cs) main" > /etc/apt/sources.list.d/dotnetdev.list'
        sudo apt-get update
        sudo apt-get install azure-functions-core-tools-4
    elif [[ "$OSTYPE" == "darwin"* ]]; then
        # macOS
        brew tap azure/functions
        brew install azure-functions-core-tools@4
    else
        echo "⚠️ Please install Azure Functions Core Tools manually"
        echo "Visit: https://docs.microsoft.com/en-us/azure/azure-functions/functions-run-local"
        exit 1
    fi
fi

echo ""
echo "🚀 Deploying Complete Pipeline to func-bim-processor..."
echo "This may take several minutes..."
echo ""

# Deploy the function app
func azure functionapp publish func-bim-processor --python

echo ""
echo "✅ Complete Pipeline deployment completed!"
echo ""
echo "🎯 Your Complete Function App now includes:"
echo ""
echo "📋 ENHANCED PHASE 1 FUNCTIONS:"
echo "   • ModelUploader - DTDL schema deployment with CommandRequest"
echo "   • UploadBIM - IFC processing with semantic name extraction"
echo "   • RobotStatus - Fleet monitoring with semantic building data"
echo ""
echo "📋 WORKING PHASE 2 FUNCTIONS:"
echo "   • TaskPlanner - AI-powered construction task planning"
echo "   • PromptOrchestrator - Chain-of-thought reasoning"
echo "   • TaskOutputGenerator - ROS2-compatible output"
echo ""
echo "🔗 Test your Complete Pipeline endpoints:"
echo ""
echo "Enhanced Phase 1 Tests:"
echo "curl \"https://func-bim-processor-a9f3hqf4bmgfhtbu.westcentralus-01.azurewebsites.net/api/ModelUploader?code=ckeNuUHMxwGw77l32DjyqHHJh3P_d319t83tTlUjF8BjAzFuF_89zQ==\""
echo ""
echo "Working Phase 2 Tests:"
echo "curl \"https://func-bim-processor-a9f3hqf4bmgfhtbu.westcentralus-01.azurewebsites.net/api/TaskPlanner?building_id=building-test&code=ckeNuUHMxwGw77l32DjyqHHJh3P_d319t83tTlUjF8BjAzFuF_89zQ==\""
echo ""
echo "⚠️ IMPORTANT: Verify AZURE_OPENAI_API_KEY is set in Function App settings!"
echo "   1. Go to Azure Portal → func-bim-processor"
echo "   2. Settings → Configuration"
echo "   3. Add: AZURE_OPENAI_API_KEY = 0d0ad883aff34d9d8cbfaef41a2c9b0b"
echo ""
echo "🧪 Run complete test suite:"
echo "python3 test_complete_pipeline.py"
echo ""
echo "🎉 Complete Pipeline Ready!"
echo "📋 Phase 1 ✅ Enhanced: Semantic names + Area info + CommandRequest"
echo "📋 Phase 2 ✅ Working: AI Planning + Chain-of-Thought + ROS2 Output"
echo "📋 Next: Phase 3 - Robot Simulation (ROS2 + Gazebo)"
