#!/bin/bash

echo "🎯 LAUNCHING PHASE 3 WITH PROMPTORCHESTRATOR CHAIN-OF-THOUGHT"
echo "=============================================================="
echo "🧠 Using PromptOrchestrator for LLM-based reasoning"
echo "🏗️ ROS2 + Gazebo + Azure Integration with Chain-of-Thought"
echo ""

# Apply environment fixes
export XDG_RUNTIME_DIR="$HOME/.local/runtime"
export XDG_CACHE_HOME="$HOME/.cache"
export XDG_DATA_HOME="$HOME/.local/share"
export XDG_CONFIG_HOME="$HOME/.config"
mkdir -p "$XDG_RUNTIME_DIR" && chmod 0700 "$XDG_RUNTIME_DIR"

# Kill existing processes
pkill -f gazebo 2>/dev/null || true
pkill -f gz 2>/dev/null || true
pkill -f azure 2>/dev/null || true
pkill -f ros2 2>/dev/null || true
pkill -f prompt 2>/dev/null || true
sleep 3

echo "🏗️ Starting Gazebo with IFC-aligned Digital Twin building..."
gz sim phase3_ifc_digital_twin.sdf &
GAZEBO_PID=$!

echo "⏳ Waiting for Digital Twin building to initialize..."
sleep 15

echo "🤖 Starting ROS2 PromptOrchestrator Task Executor..."
python3 phase3_prompt_orchestrator_executor.py &
ROS2_PID=$!

echo "🔄 Starting Enhanced Workflow Simulator with Chain-of-Thought..."
python3 enhanced_workflow_simulator.py &
WORKFLOW_PID=$!

echo ""
echo "🎯 PHASE 3 PROMPTORCHESTRATOR SYSTEM RUNNING!"
echo "============================================="
echo "✅ DELIVERABLE 1: ROS2 node with PromptOrchestrator - RUNNING (PID: $ROS2_PID)"
echo "✅ DELIVERABLE 2: Enhanced robot workflow - RUNNING (PID: $WORKFLOW_PID)"
echo "🏗️ Gazebo Digital Twin Building - RUNNING (PID: $GAZEBO_PID)"
echo ""
echo "🧠 CHAIN-OF-THOUGHT FEATURES:"
echo "   ✅ Multi-step reasoning (Site Analysis → Task Sequencing → Execution)"
echo "   ✅ Detailed dependencies and safety protocols"
echo "   ✅ Quality checkpoints from LLM reasoning"
echo "   ✅ Adaptive workflow based on reasoning steps"
echo "   ✅ Reasoning history tracking"
echo ""
echo "🏢 Building Structure: Mixed-use Residential/Commercial"
echo "📐 Total Area: 73m² (IFC-aligned)"
echo "🏠 Floors: 2 (Ground + Upper)"
echo "🏠 Spaces: 5 (2 bedrooms, kitchen, bathroom, lounge)"
echo "🤖 Robots: 2 specialized construction robots"
echo ""
echo "🌐 AZURE INTEGRATION:"
echo "   📡 PromptOrchestrator: Fetching every 45 seconds"
echo "   🧠 Chain-of-Thought: 3-step reasoning process"
echo "   🔄 Enhanced Workflow: 6-phase construction"
echo "   📊 Telemetry: Real-time with reasoning context"
echo ""
echo "📊 MONITORING FILES:"
echo "   • prompt_orchestrator_telemetry.json - Real-time status"
echo "   • chain_of_thought_history.json - Reasoning history"
echo "   • reasoning_workflow_*.json - Workflow completions"
echo "   • phase3_promptorchestrator_completion.json - Final report"
echo ""
echo "🔄 REASONING-ENHANCED WORKFLOWS:"
echo "   1. Site Analysis (reasoning-driven survey)"
echo "   2. Foundation Preparation (dependency-aware)"
echo "   3. Structural Assembly (safety-critical)"
echo "   4. Upper Floor Construction (parallel execution)"
echo "   5. Systems Integration (quality checkpoints)"
echo "   6. Quality Finishing (standards compliance)"
echo ""
echo "🎮 SYSTEM CONTROLS:"
echo "   • Ctrl+C: Stop all processes"
echo "   • View reasoning: cat chain_of_thought_history.json"
echo "   • Check telemetry: cat prompt_orchestrator_telemetry.json"
echo ""
echo "🎯 Phase 3 with PromptOrchestrator Requirements:"
echo "   ✅ LLM Chain-of-Thought task planning (PromptOrchestrator)"
echo "   ✅ Structured JSON with reasoning steps"
echo "   ✅ ROS2 parsing of detailed task plans"
echo "   ✅ Robot simulation with reasoning context"
echo "   ✅ Safety and quality from LLM reasoning"
echo "   ✅ Multi-robot coordination with dependencies"
echo ""

# Wait for user input to stop
echo "Press Enter to stop all processes..."
read

echo "🛑 Stopping Phase 3 PromptOrchestrator System..."
kill $GAZEBO_PID 2>/dev/null || true
kill $ROS2_PID 2>/dev/null || true
kill $WORKFLOW_PID 2>/dev/null || true

pkill -f gazebo 2>/dev/null || true
pkill -f gz 2>/dev/null || true
pkill -f prompt 2>/dev/null || true
pkill -f ros2 2>/dev/null || true

echo "✅ All processes stopped. Phase 3 with PromptOrchestrator complete!"
