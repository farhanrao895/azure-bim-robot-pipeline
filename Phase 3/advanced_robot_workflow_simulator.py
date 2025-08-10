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

class AdvancedRobotWorkflowSimulator(Node):
    def __init__(self):
        super().__init__('advanced_robot_workflow_simulator')
        
        self.load_building_config()
        
        self.robot1_pub = self.create_publisher(Twist, '/azure_robot_01/cmd_vel', 10)
        self.robot2_pub = self.create_publisher(Twist, '/azure_robot_02/cmd_vel', 10)
        self.workflow_pub = self.create_publisher(String, '/robot_workflow_status', 10)
        
        self.workflows = {
            "foundation_preparation": {
                "steps": ["site_survey", "excavation", "foundation_laying", "curing"],
                "current_step": 0,
                "completion": 0.0,
                "spaces": ["entire_foundation"],
                "robots": ["azure_construction_robot_01", "azure_construction_robot_02"]
            },
            "ground_floor_construction": {
                "steps": ["wall_framing", "electrical_rough", "plumbing_rough", "insulation"],
                "current_step": 0,
                "completion": 0.0,
                "spaces": ["bedroom_01", "bedroom_02", "kitchen", "bathroom"],
                "robots": ["azure_construction_robot_01"]
            },
            "upper_floor_construction": {
                "steps": ["floor_joists", "subfloor", "wall_assembly", "structural_work"],
                "current_step": 0,
                "completion": 0.0,
                "spaces": ["lounge"],
                "robots": ["azure_construction_robot_02"]
            },
            "mechanical_systems": {
                "steps": ["hvac_installation", "electrical_finishing", "plumbing_finishing", "testing"],
                "current_step": 0,
                "completion": 0.0,
                "spaces": ["all_spaces"],
                "robots": ["azure_construction_robot_01", "azure_construction_robot_02"]
            },
            "interior_finishing": {
                "steps": ["drywall", "painting", "flooring", "fixtures", "final_inspection"],
                "current_step": 0,
                "completion": 0.0,
                "spaces": ["all_spaces"],
                "robots": ["azure_construction_robot_01", "azure_construction_robot_02"]
            }
        }
        
        self.current_workflow = "foundation_preparation"
        self.workflow_start_time = time.time()
        self.robot_positions = {
            "azure_construction_robot_01": {"x": -3.5, "y": -2.0, "z": 0.5},
            "azure_construction_robot_02": {"x": 4.0, "y": 1.5, "z": 4.7}
        }
        
        self.workflow_timer = self.create_timer(12.0, self.advance_workflow)
        self.robot_movement_timer = self.create_timer(3.0, self.move_robots_for_workflow)
        
        self.get_logger().info('🔄 IFC-Aligned Robot Workflow Simulator!')
        self.get_logger().info(f'🏗️ Building: {self.building_config["structure_type"]}')
        self.get_logger().info(f'📐 Total Area: {self.building_config["building_area"]}m²')

    def load_building_config(self):
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

    def advance_workflow(self):
        try:
            workflow = self.workflows[self.current_workflow]
            
            if workflow["current_step"] < len(workflow["steps"]):
                current_step_name = workflow["steps"][workflow["current_step"]]
                
                workflow["completion"] += 20.0
                
                if workflow["completion"] >= 100.0:
                    workflow["completion"] = 100.0
                    workflow["current_step"] += 1
                    
                    if workflow["current_step"] >= len(workflow["steps"]):
                        self.complete_current_workflow()
                    else:
                        workflow["completion"] = 0.0
                        self.get_logger().info(f'🔄 Starting step: {workflow["steps"][workflow["current_step"]]}')
                
                self.publish_workflow_status(current_step_name, workflow["completion"])
                
            else:
                self.advance_to_next_workflow()
                
        except Exception as e:
            self.get_logger().error(f'Error advancing workflow: {str(e)}')

    def move_robots_for_workflow(self):
        try:
            workflow = self.workflows[self.current_workflow]
            robots_for_workflow = workflow["robots"]
            spaces = workflow["spaces"]
            
            for robot_name in robots_for_workflow:
                if robot_name in self.robot_positions:
                    self.move_robot_for_current_task(robot_name, spaces)
                    
        except Exception as e:
            self.get_logger().error(f'Error moving robots: {str(e)}')

    def move_robot_for_current_task(self, robot_name, spaces):
        try:
            if robot_name == "azure_construction_robot_01":
                if "bedroom_01" in spaces:
                    target_x, target_y, target_z = -6.0, -3.0, 0.5
                elif "bedroom_02" in spaces:
                    target_x, target_y, target_z = -1.5, -3.0, 0.5
                elif "kitchen" in spaces:
                    target_x, target_y, target_z = 2.0, -2.5, 0.5
                elif "bathroom" in spaces:
                    target_x, target_y, target_z = -2.0, 1.0, 0.5
                else:
                    angle = time.time() * 0.2
                    target_x = -3.5 + 1.5 * math.cos(angle)
                    target_y = -2.0 + 1.0 * math.sin(angle)
                    target_z = 0.5
            else:
                if "lounge" in spaces:
                    target_x, target_y, target_z = 3.0, 1.0, 4.7
                else:
                    angle = time.time() * 0.15
                    target_x = 4.0 + 1.2 * math.sin(angle)
                    target_y = 1.5 + 0.8 * math.sin(2 * angle)
                    target_z = 4.7
            
            success = self.move_robot_gazebo(robot_name, target_x, target_y, target_z)
            
            if success:
                self.robot_positions[robot_name] = {"x": target_x, "y": target_y, "z": target_z}
                
                cmd = Twist()
                if robot_name == "azure_construction_robot_01":
                    cmd.linear.x = 0.2
                    cmd.angular.z = 0.1
                    self.robot1_pub.publish(cmd)
                else:
                    cmd.linear.x = 0.15
                    cmd.angular.z = 0.05
                    self.robot2_pub.publish(cmd)
            
        except Exception as e:
            self.get_logger().error(f'Error moving robot {robot_name}: {str(e)}')

    def move_robot_gazebo(self, robot_name, x, y, z):
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

    def publish_workflow_status(self, step_name, completion):
        workflow = self.workflows[self.current_workflow]
        
        status = {
            "workflow": self.current_workflow,
            "step": step_name,
            "completion": completion,
            "timestamp": datetime.now().isoformat(),
            "building_context": {
                "structure_type": self.building_config["structure_type"],
                "total_area": self.building_config["building_area"],
                "total_floors": self.building_config.get("total_floors", 2),
                "affected_spaces": workflow["spaces"],
                "robots_assigned": workflow["robots"]
            },
            "robot_positions": self.robot_positions
        }
        
        msg = String()
        msg.data = json.dumps(status)
        self.workflow_pub.publish(msg)
        
        self.get_logger().info(f'🔄 Workflow: {self.current_workflow} | Step: {step_name} | Progress: {completion:.1f}%')
        self.get_logger().info(f'🏗️ IFC Spaces: {workflow["spaces"]} | Robots: {len(workflow["robots"])}')

    def complete_current_workflow(self):
        completion_time = time.time() - self.workflow_start_time
        
        completion_data = {
            "workflow": self.current_workflow,
            "completion_time": datetime.now().isoformat(),
            "duration_seconds": completion_time,
            "building_context": {
                "structure_type": self.building_config["structure_type"],
                "spaces_completed": self.workflows[self.current_workflow]["spaces"],
                "robots_used": self.workflows[self.current_workflow]["robots"]
            }
        }
        
        with open(f'workflow_completion_{self.current_workflow}.json', 'w') as f:
            json.dump(completion_data, f, indent=2)
            
        self.get_logger().info(f'✅ Workflow completed: {self.current_workflow} ({completion_time:.1f}s)')

    def advance_to_next_workflow(self):
        workflow_sequence = [
            "foundation_preparation", 
            "ground_floor_construction", 
            "upper_floor_construction", 
            "mechanical_systems", 
            "interior_finishing"
        ]
        
        try:
            current_index = workflow_sequence.index(self.current_workflow)
            if current_index + 1 < len(workflow_sequence):
                self.current_workflow = workflow_sequence[current_index + 1]
                self.workflow_start_time = time.time()
                self.get_logger().info(f'🔄 Starting new workflow: {self.current_workflow}')
                
                self.workflows[self.current_workflow]["current_step"] = 0
                self.workflows[self.current_workflow]["completion"] = 0.0
            else:
                self.get_logger().info('🎉 All IFC Digital Twin building construction workflows completed!')
                self.complete_all_workflows()
                
        except ValueError:
            self.get_logger().error(f'Unknown workflow: {self.current_workflow}')

    def complete_all_workflows(self):
        total_duration = time.time() - self.workflow_start_time
        
        final_report = {
            "project_status": "COMPLETED",
            "completion_time": datetime.now().isoformat(),
            "total_duration_seconds": total_duration,
            "building_details": {
                "structure_type": self.building_config["structure_type"],
                "total_area": self.building_config["building_area"],
                "total_floors": self.building_config.get("total_floors", 2),
                "spaces_constructed": [
                    "bedroom_01", "bedroom_02", "kitchen", "bathroom", "lounge"
                ]
            },
            "workflows_completed": list(self.workflows.keys()),
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
        
        with open('phase3_ifc_completion_report.json', 'w') as f:
            json.dump(final_report, f, indent=2)
        
        self.get_logger().info('🎉 Phase 3 IFC-Aligned Complete - All building workflows finished!')
        self.get_logger().info(f'📊 Total Duration: {total_duration:.1f} seconds')
        self.get_logger().info(f'🏗️ Building Area: {self.building_config["building_area"]}m²')

def main(args=None):
    rclpy.init(args=args)
    simulator = AdvancedRobotWorkflowSimulator()
    
    try:
        rclpy.spin(simulator)
    except KeyboardInterrupt:
        pass
    finally:
        simulator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
