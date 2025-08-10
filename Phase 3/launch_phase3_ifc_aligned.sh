#!/bin/bash

echo "🎯 LAUNCHING PHASE 3 IFC-ALIGNED DIGITAL TWIN SYSTEM"
echo "=================================================="
echo "🏗️ ROS2 + Gazebo + Azure Integration - Matching IFC Model Exactly"
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
sleep 3

echo "🏗️ Starting Gazebo with IFC-aligned Digital Twin building simulation..."
gz sim phase3_ifc_digital_twin.sdf &
GAZEBO_PID=$!

echo "⏳ Waiting for IFC Digital Twin building to initialize..."
sleep 15

echo "🤖 Starting ROS2 IFC-Aligned Task Executor..."
python3 phase3_advanced_task_executor.py &
ROS2_PID=$!

echo "🔄 Starting IFC-Aligned Robot Workflow Simulator..."
python3 advanced_robot_workflow_simulator.py &
WORKFLOW_PID=$!

echo ""
echo "🎯 PHASE 3 IFC-ALIGNED DIGITAL TWIN SYSTEM RUNNING!"
echo "==============================================="
echo "✅ DELIVERABLE 1: ROS2 node to simulate task - RUNNING (PID: $ROS2_PID)"
echo "✅ DELIVERABLE 2: Simulated robot workflow - RUNNING (PID: $WORKFLOW_PID)"
echo "🏗️ Gazebo IFC Digital Twin Building - RUNNING (PID: $GAZEBO_PID)"
echo ""
echo "Press Enter to stop all processes..."
read

echo "🛑 Stopping Phase 3 IFC-Aligned Digital Twin System..."
kill $GAZEBO_PID 2>/dev/null || true
kill $ROS2_PID 2>/dev/null || true
kill $WORKFLOW_PID 2>/dev/null || true

pkill -f gazebo 2>/dev/null || true
pkill -f gz 2>/dev/null || true
pkill -f azure 2>/dev/null || true
pkill -f ros2 2>/dev/null || true

echo "✅ All processes stopped. Phase 3 system shutdown complete!"
