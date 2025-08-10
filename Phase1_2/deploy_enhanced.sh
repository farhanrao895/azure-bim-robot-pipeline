#!/bin/bash
# Enhanced Pipeline Deployment with Security Improvements

echo "🚀 ENHANCED Azure BIM-Robot Pipeline Deployment"
echo "==============================================="
echo "✅ Multi-floor navigation support"
echo "✅ Semantic space targeting"
echo "✅ Floor-aware task distribution"
echo "✅ Security improvements"
echo ""

# Check if API key is set in environment
if [ -z "$AZURE_OPENAI_API_KEY" ]; then
    echo "⚠️ WARNING: AZURE_OPENAI_API_KEY not set in environment"
    echo "Please set it in Azure Portal -> Function App -> Configuration"
    echo "DO NOT hardcode API keys in scripts!"
else
    echo "✅ API key found in environment"
fi

# Deploy the function app
echo "Deploying to Azure..."
func azure functionapp publish func-bim-processor --python

echo ""
echo "✅ Enhanced Pipeline deployment completed!"
echo ""
echo "⚠️ IMPORTANT SECURITY NOTES:"
echo "1. Set AZURE_OPENAI_API_KEY in Azure Portal Configuration (not in code)"
echo "2. Rotate any exposed API keys immediately"
echo "3. Use Azure Key Vault for production deployments"
echo ""
echo "🔍 Key Enhancements Applied:"
echo "• Multi-floor navigation with proper z-coordinates"
echo "• Semantic space name targeting"
echo "• Floor-aware task distribution"
echo "• At least 2 waypoints per navigation task"
echo "• Security improvements for API key handling"
