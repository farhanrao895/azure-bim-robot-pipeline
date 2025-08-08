#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import json
import time
import math
import subprocess
from datetime import datetime

class EnhancedWorkflowSimulator(Node):
    """Enhanced Robot Workflow Simulator with Chain-of-Thought Integration"""
    
    def __init__(self):
        super().__init__('enhanced_workflow_simulator')
        
        self.load_building_config()
        
        # Publishers
        self.robot1_pub = self.create_publisher(Twist, '/azure_robot_01/cmd_vel', 10)
        self.robot2_pub = self.create_publisher(Twist, '/azure_robot_02/cmd_vel', 10)
        self.workflow_pub = self.create_publisher(String, '/robot_workflow_status', 10)
        
        # Subscribe to reasoning steps
        self.reasoning_sub = self.create_subscription(
            String,
            '/llm/reasoning_steps',
            self.handle_reasoning_update,
            10
        )
        
        # Enhanced workflows with reasoning-based adaptation
        self.workflows = {
            "site_analysis": {
                "steps": ["site_survey", "accessibility_check", "safety_assessment", "resource_planning"],
                "current_step": 0,
                "completion": 0.0,
                "spaces": ["entire_site"],
                "robots": ["azure_construction_robot_01", "azure_construction_robot_02"],
                "reasoning_driven": True,
                "adaptable": True
            },
            "foundation_preparation": {
                "steps": ["excavation_planning", "soil_preparation", "foundation_marking", "reinforcement_placement"],
                "current_step": 0,
                "completion": 0.0,
                "spaces": ["bedroom_01", "bedroom_02", "kitchen", "bathroom"],
                "robots": ["azure_construction_robot_01"],
                "reasoning_driven": True,
                "dependencies": []
            },
            "structural_assembly": {
                "steps": ["column_placement", "beam_installation", "floor_slab", "wall_framing"],
                "current_step": 0,
                "completion": 0.0,
                "spaces": ["all_ground_floor"],
                "robots": ["azure_construction_robot_01"],
                "reasoning_driven": True,
                "quality_checkpoints": ["alignment", "levelness", "stability"]
            },
            "upper_floor_construction": {
                "steps": ["floor_preparation", "structural_elements", "safety_barriers", "utilities_rough"],
                "current_step": 0,
                "completion": 0.0,
                "spaces": ["lounge"],
                "robots": ["azure_construction_robot_02"],
                "reasoning_driven": True,
                "parallel_execution": True
            },
            "systems_integration": {
                "steps": ["electrical_wiring", "plumbing_installation", "hvac_setup", "system_testing"],
                "current_step": 0,
                "completion": 0.0,
                "spaces": ["all_spaces"],
                "robots": ["azure_construction_robot_01", "azure_construction_robot_02"],
                "reasoning_driven": True,
                "safety_critical": True
            },
            "quality_finishing": {
                "steps": ["surface_preparation", "painting", "fixture_installation", "final_inspection", "certification"],
                "current_step": 0,
                "completion": 0.0,
                "spaces": ["all_spaces"],
                "robots": ["azure_construction_robot_01", "azure_construction_robot_02"],
                "reasoning_driven": True,
                "quality_standards": ["iso9001", "safety_compliance"]
            }
        }
        
        self.current_workflow = "site_analysis"
        self.workflow_start_time = time.time()
        self.reasoning_context = None
        self.adaptive_scheduling = True
        
        self.robot_positions = {
            "azure_construction_robot_01": {"x": -3.5, "y": -2.0, "z": 0.5},
            "azure_construction_robot_02": {"x": 4.0, "y": 1.5, "z": 4.7}
        }
        
        # Timers
        self.workflow_timer = self.create_timer(10.0, self.advance_workflow_with_reasoning)
        self.robot_movement_timer = self.create_timer(3.0, self.move_robots_with_reasoning)
        
        self.get_logger().info('🔄 Enhanced Workflow Simulator with Chain-of-Thought!')
        self.get_logger().info(f'🏗️ Building: {self.building_config["structure_type"]}')
        self.get_logger().info(f'🧠 Reasoning-driven workflows: ENABLED')

    def load_building_config(self):
        """Load building configuration"""
        try:
            with open('azure_config_advanced.json', 'r') as f:
                config = json.load(f)
            self.building_config = config["digital_twin_building"]
            self.building_config["building_area"] = config["world_settings"]["building_area"]
            self.world_name = config["world_settings"]["world_name"]
        except:
            self.building_config = {
                "structure_type": "mixed_use_residential_commercial",
                "building_area": 73,
                "total_floors": 2
            }
            self.world_name = "phase3_ifc_digital_twin"

    def handle_reasoning_update(self, msg):
        """Handle reasoning updates from PromptOrchestrator"""
        try:
            reasoning_data = json.loads(msg.data)
            self.reasoning_context = reasoning_data
            
            # Adapt workflow based on reasoning
            step_title = reasoning_data.get("title", "").lower()
            
            if "site" in step_title or "analysis" in step_title:
                self.prioritize_workflow("site_analysis")
            elif "foundation" in step_title:
                self.prioritize_workflow("foundation_preparation")
            elif "structural" in step_title or "frame" in step_title:
                self.prioritize_workflow("structural_assembly")
            elif "systems" in step_title or "mep" in step_title:
                self.prioritize_workflow("systems_integration")
            elif "finishing" in step_title or "quality" in step_title:
                self.prioritize_workflow("quality_finishing")
                
            self.get_logger().info(f'🧠 Workflow adapted based on reasoning: {step_title}')
            
        except Exception as e:
            self.get_logger().error(f'Error handling reasoning: {str(e)}')

    def prioritize_workflow(self, workflow_name):
        """Prioritize a specific workflow based on reasoning"""
        if workflow_name in self.workflows and workflow_name != self.current_workflow:
            # Save current workflow state
            self.workflows[self.current_workflow]["paused"] = True
            
            # Switch to prioritized workflow
            self.current_workflow = workflow_name
            self.workflow_start_time = time.time()
            
            self.get_logger().info(f'🔄 Switched to workflow: {workflow_name} (reasoning-driven)')

    def advance_workflow_with_reasoning(self):
        """Advance workflow with Chain-of-Thought consideration"""
        try:
            workflow = self.workflows[self.current_workflow]
            
            # Check if workflow should adapt based on reasoning
            if workflow.get("reasoning_driven") and self.reasoning_context:
                self.adapt_workflow_steps(workflow)
            
            if workflow["current_step"] < len(workflow["steps"]):
                current_step_name = workflow["steps"][workflow["current_step"]]
                
                # Adaptive completion rate based on reasoning
                completion_rate = 25.0 if workflow.get("reasoning_driven") else 20.0
                workflow["completion"] += completion_rate
                
                if workflow["completion"] >= 100.0:
                    workflow["completion"] = 100.0
                    workflow["current_step"] += 1
                    
                    if workflow["current_step"] >= len(workflow["steps"]):
                        self.complete_workflow_with_reasoning()
                    else:
                        workflow["completion"] = 0.0
                        next_step = workflow["steps"][workflow["current_step"]]
                        self.get_logger().info(f'🔄 Starting step: {next_step} (reasoning-enhanced)')
                
                self.publish_workflow_status_with_reasoning(current_step_name, workflow["completion"])
                
            else:
                self.advance_to_next_workflow_with_reasoning()
                
        except Exception as e:
            self.get_logger().error(f'Error advancing workflow: {str(e)}')

    def adapt_workflow_steps(self, workflow):
        """Adapt workflow steps based on Chain-of-Thought reasoning"""
        if not self.reasoning_context:
            return
            
        reasoning_text = self.reasoning_context.get("reasoning", "").lower()
        
        # Add safety checks if mentioned in reasoning
        if "safety" in reasoning_text and "safety_verification" not in workflow["steps"]:
            current_step = workflow["current_step"]
            workflow["steps"].insert(current_step + 1, "safety_verification")
            self.get_logger().info('➕ Added safety verification step based on reasoning')
        
        # Add quality checks if mentioned
        if "quality" in reasoning_text and "quality_checkpoint" not in workflow["steps"]:
            current_step = workflow["current_step"]
            workflow["steps"].insert(current_step + 1, "quality_checkpoint")
            self.get_logger().info('➕ Added quality checkpoint based on reasoning')
        
        # Adjust for dependencies mentioned in reasoning
        if "dependency" in reasoning_text or "prerequisite" in reasoning_text:
            workflow["dependencies"] = workflow.get("dependencies", [])
            workflow["dependencies"].append(f"reasoning_step_{self.reasoning_context.get('step_number', 0)}")

    def move_robots_with_reasoning(self):
        """Move robots based on current workflow with reasoning context"""
        try:
            workflow = self.workflows[self.current_workflow]
            robots_for_workflow = workflow["robots"]
            spaces = workflow["spaces"]
            
            for robot_name in robots_for_workflow:
                if robot_name in self.robot_positions:
                    self.move_robot_for_reasoning_task(robot_name, spaces, workflow)
                    
        except Exception as e:
            self.get_logger().error(f'Error moving robots: {str(e)}')

    def move_robot_for_reasoning_task(self, robot_name, spaces, workflow):
        """Move robot based on reasoning-enhanced task requirements"""
        try:
            # Determine target based on workflow and reasoning
            if self.reasoning_context:
                target_coords = self.get_reasoning_based_target(robot_name, workflow)
            else:
                target_coords = self.get_default_target(robot_name, spaces)
            
            target_x, target_y, target_z = target_coords
            
            success = self.move_robot_gazebo(robot_name, target_x, target_y, target_z)
            
            if success:
                self.robot_positions[robot_name] = {"x": target_x, "y": target_y, "z": target_z}
                
                # Publish movement command
                cmd = Twist()
                if robot_name == "azure_construction_robot_01":
                    cmd.linear.x = 0.25  # Slightly faster for reasoning-driven tasks
                    cmd.angular.z = 0.1
                    self.robot1_pub.publish(cmd)
                else:
                    cmd.linear.x = 0.2
                    cmd.angular.z = 0.05
                    self.robot2_pub.publish(cmd)
            
        except Exception as e:
            self.get_logger().error(f'Error moving robot {robot_name}: {str(e)}')

    def get_reasoning_based_target(self, robot_name, workflow):
        """Get target coordinates based on reasoning context"""
        step_number = self.reasoning_context.get("step_number", 1)
        
        if robot_name == "azure_construction_robot_01":
            # Ground floor movements adapted by reasoning step
            if step_number == 1:  # Site analysis
                angle = time.time() * 0.1
                return (-6.0 + 3 * math.cos(angle), -3.0 + 2 * math.sin(angle), 0.5)
            elif step_number == 2:  # Task sequencing
                return (-1.5, -3.0, 0.5)  # Bedroom 02
            else:  # Final execution
                return (2.0, -2.5, 0.5)  # Kitchen
        else:
            # Upper floor movements
            if workflow.get("parallel_execution"):
                angle = time.time() * 0.15
                return (3.0 + 1.5 * math.cos(angle), 1.0 + 1.5 * math.sin(angle), 4.7)
            else:
                return (3.0, 1.0, 4.7)  # Lounge center

    def get_default_target(self, robot_name, spaces):
        """Get default target when no reasoning context available"""
        if robot_name == "azure_construction_robot_01":
            if "bedroom_01" in str(spaces):
                return (-6.0, -3.0, 0.5)
            elif "kitchen" in str(spaces):
                return (2.0, -2.5, 0.5)
            else:
                angle = time.time() * 0.2
                return (-3.5 + 1.5 * math.cos(angle), -2.0 + 1.0 * math.sin(angle), 0.5)
        else:
            return (3.0, 1.0, 4.7)

    def move_robot_gazebo(self, robot_name, x, y, z):
        """Move robot in Gazebo"""
        try:
            cmd = [
                'gz', 'service', '-s', f'/world/{self.world_name}/set_pose',
                '--reqtype', 'gz.msgs.Pose',
                '--reptype', 'gz.msgs.Boolean',
                '--timeout', '1000',
                '--req', 
                f'name: "{robot_name}" position: {{x: {x}, y: {y}, z: {z}}} orientation: {{x: 0, y: 0, z: 0, w: 1}}'
            ]
            
            result = subprocess.run(cmd, capture_output=True, timeout=2, text=True)
            return result.returncode == 0
            
        except:
            return False

    def publish_workflow_status_with_reasoning(self, step_name, completion):
        """Publish workflow status with reasoning context"""
        workflow = self.workflows[self.current_workflow]
        
        status = {
            "workflow": self.current_workflow,
            "step": step_name,
            "completion": completion,
            "timestamp": datetime.now().isoformat(),
            "reasoning_enhanced": workflow.get("reasoning_driven", False),
            "reasoning_context": {
                "step_number": self.reasoning_context.get("step_number", 0) if self.reasoning_context else 0,
                "title": self.reasoning_context.get("title", "") if self.reasoning_context else "",
                "adapted": workflow.get("adapted", False)
            },
            "building_context": {
                "structure_type": self.building_config["structure_type"],
                "total_area": self.building_config["building_area"],
                "total_floors": self.building_config.get("total_floors", 2),
                "affected_spaces": workflow["spaces"],
                "robots_assigned": workflow["robots"]
            },
            "quality_metrics": {
                "safety_checks": workflow.get("safety_critical", False),
                "quality_standards": workflow.get("quality_standards", []),
                "dependencies_met": len(workflow.get("dependencies", [])) == 0
            },
            "robot_positions": self.robot_positions
        }
        
        msg = String()
        msg.data = json.dumps(status)
        self.workflow_pub.publish(msg)
        
        self.get_logger().info(f'🔄 Workflow: {self.current_workflow} | Step: {step_name} | Progress: {completion:.1f}%')
        if workflow.get("reasoning_driven"):
            self.get_logger().info(f'   🧠 Reasoning-enhanced execution active')

    def complete_workflow_with_reasoning(self):
        """Complete current workflow with reasoning summary"""
        completion_time = time.time() - self.workflow_start_time
        
        completion_data = {
            "workflow": self.current_workflow,
            "completion_time": datetime.now().isoformat(),
            "duration_seconds": completion_time,
            "reasoning_enhanced": True,
            "reasoning_steps_applied": self.reasoning_context is not None,
            "building_context": {
                "structure_type": self.building_config["structure_type"],
                "spaces_completed": self.workflows[self.current_workflow]["spaces"],
                "robots_used": self.workflows[self.current_workflow]["robots"]
            },
            "quality_metrics": {
                "adaptations_made": self.workflows[self.current_workflow].get("adapted", False),
                "safety_verified": self.workflows[self.current_workflow].get("safety_critical", False),
                "standards_met": self.workflows[self.current_workflow].get("quality_standards", [])
            }
        }
        
        with open(f'reasoning_workflow_{self.current_workflow}.json', 'w') as f:
            json.dump(completion_data, f, indent=2)
            
        self.get_logger().info(f'✅ Workflow completed: {self.current_workflow} ({completion_time:.1f}s)')
        self.get_logger().info(f'   🧠 Reasoning-enhanced: YES')

    def advance_to_next_workflow_with_reasoning(self):
        """Advance to next workflow with reasoning consideration"""
        workflow_sequence = [
            "site_analysis",
            "foundation_preparation",
            "structural_assembly",
            "upper_floor_construction",
            "systems_integration",
            "quality_finishing"
        ]
        
        try:
            current_index = workflow_sequence.index(self.current_workflow)
            if current_index + 1 < len(workflow_sequence):
                self.current_workflow = workflow_sequence[current_index + 1]
                self.workflow_start_time = time.time()
                self.get_logger().info(f'🔄 Starting workflow: {self.current_workflow} (reasoning-aware)')
                
                # Reset workflow
                self.workflows[self.current_workflow]["current_step"] = 0
                self.workflows[self.current_workflow]["completion"] = 0.0
                self.workflows[self.current_workflow]["adapted"] = False
            else:
                self.get_logger().info('🎉 All reasoning-enhanced workflows completed!')
                self.complete_all_workflows_with_reasoning()
                
        except ValueError:
            self.get_logger().error(f'Unknown workflow: {self.current_workflow}')

    def complete_all_workflows_with_reasoning(self):
        """Complete all workflows with comprehensive reasoning report"""
        total_duration = sum(
            time.time() - self.workflow_start_time 
            for workflow in self.workflows.values()
        )
        
        final_report = {
            "project_status": "COMPLETED_WITH_CHAIN_OF_THOUGHT",
            "completion_time": datetime.now().isoformat(),
            "total_duration_seconds": total_duration,
            "orchestration_type": "PromptOrchestrator_Enhanced",
            "building_details": {
                "structure_type": self.building_config["structure_type"],
                "total_area": self.building_config["building_area"],
                "total_floors": self.building_config.get("total_floors", 2),
                "spaces_constructed": [
                    "bedroom_01", "bedroom_02", "kitchen", "bathroom", "lounge"
                ]
            },
            "workflows_completed": list(self.workflows.keys()),
            "reasoning_integration": {
                "chain_of_thought_applied": True,
                "adaptive_scheduling": self.adaptive_scheduling,
                "reasoning_driven_workflows": sum(
                    1 for w in self.workflows.values() if w.get("reasoning_driven")
                ),
                "total_adaptations": sum(
                    1 for w in self.workflows.values() if w.get("adapted", False)
                )
            },
            "robots_utilized": ["azure_construction_robot_01", "azure_construction_robot_02"],
            "ifc_alignment": {
                "floors": 2,
                "rooms": 5,
                "walls": 18,
                "doors": 4,
                "windows": 5,
                "slabs": 2
            }
        }
        
        with open('phase3_promptorchestrator_completion.json', 'w') as f:
            json.dump(final_report, f, indent=2)
        
        self.get_logger().info('🎉 Phase 3 Chain-of-Thought Complete!')
        self.get_logger().info(f'📊 Total Duration: {total_duration:.1f} seconds')
        self.get_logger().info(f'🧠 Reasoning-enhanced workflows: {sum(1 for w in self.workflows.values() if w.get("reasoning_driven"))}')

def main(args=None):
    rclpy.init(args=args)
    simulator = EnhancedWorkflowSimulator()
    
    try:
        rclpy.spin(simulator)
    except KeyboardInterrupt:
        pass
    finally:
        simulator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
