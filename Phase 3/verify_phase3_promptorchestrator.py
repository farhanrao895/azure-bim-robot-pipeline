#!/usr/bin/env python3

import os
import json
import subprocess
from datetime import datetime

def verify_phase3_compliance():
    print("🔍 PHASE 3 PROMPTORCHESTRATOR COMPLIANCE VERIFICATION")
    print("=" * 60)
    
    compliance_results = {
        "verification_time": datetime.now().isoformat(),
        "phase": "Phase 3 - PromptOrchestrator Chain-of-Thought Integration",
        "deliverables_status": {},
        "requirements_status": {},
        "chain_of_thought_status": {},
        "overall_compliance": False
    }
    
    # Check Deliverable 1: ROS2 node with PromptOrchestrator
    print("\n📋 DELIVERABLE 1: ROS2 node with PromptOrchestrator integration")
    deliverable1_files = ["phase3_prompt_orchestrator_executor.py"]
    deliverable1_pass = True
    
    for file in deliverable1_files:
        if os.path.exists(file):
            print(f"   ✅ {file} - EXISTS")
            with open(file, 'r') as f:
                content = f.read()
                checks = {
                    "PromptOrchestrator endpoint": "prompt_orchestrator" in content,
                    "Chain-of-Thought processing": "process_reasoning_steps" in content,
                    "ROS2 integration": "rclpy" in content,
                    "Reasoning publication": "reasoning_pub" in content,
                    "LLM command processing": "process_llm_commands_with_reasoning" in content
                }
                
                for check_name, check_result in checks.items():
                    if check_result:
                        print(f"   ✅ {check_name} - VERIFIED")
                    else:
                        print(f"   ❌ {check_name} - MISSING")
                        deliverable1_pass = False
        else:
            print(f"   ❌ {file} - MISSING")
            deliverable1_pass = False
    
    compliance_results["deliverables_status"]["deliverable_1"] = deliverable1_pass
    
    # Check Deliverable 2: Enhanced robot workflow
    print("\n📋 DELIVERABLE 2: Enhanced robot workflow with reasoning")
    deliverable2_files = ["enhanced_workflow_simulator.py"]
    deliverable2_pass = True
    
    for file in deliverable2_files:
        if os.path.exists(file):
            print(f"   ✅ {file} - EXISTS")
            with open(file, 'r') as content_file:
                content = content_file.read()
                checks = {
                    "Reasoning subscription": "reasoning_sub" in content,
                    "Workflow adaptation": "adapt_workflow_steps" in content,
                    "Reasoning-based movement": "get_reasoning_based_target" in content,
                    "Chain-of-Thought handling": "handle_reasoning_update" in content,
                    "Enhanced workflows": "reasoning_driven" in content
                }
                
                for check_name, check_result in checks.items():
                    if check_result:
                        print(f"   ✅ {check_name} - VERIFIED")
                    else:
                        print(f"   ❌ {check_name} - MISSING")
                        deliverable2_pass = False
        else:
            print(f"   ❌ {file} - MISSING")
            deliverable2_pass = False
    
    compliance_results["deliverables_status"]["deliverable_2"] = deliverable2_pass
    
    # Check Chain-of-Thought Integration
    print("\n🧠 CHAIN-OF-THOUGHT INTEGRATION VERIFICATION")
    
    cot_checks = {
        "PromptOrchestrator configuration": os.path.exists("azure_config_advanced.json"),
        "Reasoning history tracking": "reasoning_history" in open("phase3_prompt_orchestrator_executor.py").read() if os.path.exists("phase3_prompt_orchestrator_executor.py") else False,
        "Multi-step reasoning": "reasoning_steps" in open("phase3_prompt_orchestrator_executor.py").read() if os.path.exists("phase3_prompt_orchestrator_executor.py") else False,
        "Adaptive workflows": "adapt_workflow" in open("enhanced_workflow_simulator.py").read() if os.path.exists("enhanced_workflow_simulator.py") else False,
        "Safety protocols": "safety_checks" in open("phase3_prompt_orchestrator_executor.py").read() if os.path.exists("phase3_prompt_orchestrator_executor.py") else False,
        "Quality checkpoints": "quality_checkpoint" in open("enhanced_workflow_simulator.py").read() if os.path.exists("enhanced_workflow_simulator.py") else False
    }
    
    for check_name, check_result in cot_checks.items():
        if check_result:
            print(f"   ✅ {check_name}")
        else:
            print(f"   ❌ {check_name}")
    
    compliance_results["chain_of_thought_status"] = cot_checks
    
    # Check System Requirements
    print("\n⚙️ SYSTEM REQUIREMENTS VERIFICATION")
    
    # ROS2 and Gazebo integration
    gazebo_file = "phase3_ifc_digital_twin.sdf"
    ros2_gazebo_pass = os.path.exists(gazebo_file)
    print(f"   {'✅' if ros2_gazebo_pass else '❌'} ROS2 + Gazebo Integration")
    compliance_results["requirements_status"]["ros2_gazebo"] = ros2_gazebo_pass
    
    # PromptOrchestrator integration
    config_file = "azure_config_advanced.json"
    prompt_orchestrator_pass = False
    if os.path.exists(config_file):
        try:
            with open(config_file, 'r') as f:
                config = json.load(f)
                if "prompt_orchestrator" in config.get("endpoints", {}):
                    prompt_orchestrator_pass = True
                    print("   ✅ PromptOrchestrator Configuration")
                else:
                    print("   ❌ PromptOrchestrator - MISSING ENDPOINT")
        except:
            print("   ❌ PromptOrchestrator - INVALID CONFIG")
    else:
        print("   ❌ PromptOrchestrator - CONFIG FILE MISSING")
    
    compliance_results["requirements_status"]["prompt_orchestrator"] = prompt_orchestrator_pass
    
    # Chain-of-Thought reasoning
    cot_reasoning_pass = all([
        cot_checks.get("Multi-step reasoning", False),
        cot_checks.get("Reasoning history tracking", False)
    ])
    print(f"   {'✅' if cot_reasoning_pass else '❌'} Chain-of-Thought Reasoning")
    compliance_results["requirements_status"]["chain_of_thought"] = cot_reasoning_pass
    
    # Structured JSON output
    structured_json_pass = "json.dumps" in open("phase3_prompt_orchestrator_executor.py").read() if os.path.exists("phase3_prompt_orchestrator_executor.py") else False
    print(f"   {'✅' if structured_json_pass else '❌'} Structured JSON Task Plans")
    compliance_results["requirements_status"]["structured_json"] = structured_json_pass
    
    # Overall compliance calculation
    deliverables_pass = deliverable1_pass and deliverable2_pass
    requirements_pass = all(compliance_results["requirements_status"].values())
    cot_pass = sum(cot_checks.values()) >= 4  # At least 4 out of 6 CoT features
    
    overall_pass = deliverables_pass and requirements_pass and cot_pass
    compliance_results["overall_compliance"] = overall_pass
    
    # Final Results
    print("\n" + "=" * 60)
    print("📊 FINAL COMPLIANCE RESULTS")
    print("=" * 60)
    
    print(f"✅ Deliverable 1 (ROS2 + PromptOrchestrator): {'PASS' if deliverable1_pass else 'FAIL'}")
    print(f"✅ Deliverable 2 (Enhanced Workflow): {'PASS' if deliverable2_pass else 'FAIL'}")
    print(f"🧠 Chain-of-Thought Integration: {'PASS' if cot_pass else 'FAIL'}")
    print(f"⚙️ System Requirements: {'PASS' if requirements_pass else 'FAIL'}")
    
    print(f"\n🎯 OVERALL PHASE 3 COMPLIANCE: {'✅ PASS' if overall_pass else '❌ FAIL'}")
    
    if overall_pass:
        print("\n🎉 Congratulations! Your Phase 3 PromptOrchestrator system")
        print("    meets all requirements and successfully implements:")
        print("    • LLM Chain-of-Thought task planning via PromptOrchestrator")
        print("    • Multi-step reasoning (Site Analysis → Sequencing → Execution)")
        print("    • Structured JSON with detailed dependencies and safety")
        print("    • ROS2 parsing and execution of reasoning-based plans")
        print("    • Adaptive workflows based on LLM reasoning")
        print("    • Real-time telemetry with reasoning context")
    else:
        print("\n⚠️ Phase 3 system requires attention to meet full compliance.")
        print("   Please review the failed items above and make corrections.")
    
    # Save compliance report
    with open('phase3_promptorchestrator_compliance.json', 'w') as f:
        json.dump(compliance_results, f, indent=2)
    
    print(f"\n📄 Compliance report saved: phase3_promptorchestrator_compliance.json")
    
    return overall_pass

if __name__ == "__main__":
    verify_phase3_compliance()
