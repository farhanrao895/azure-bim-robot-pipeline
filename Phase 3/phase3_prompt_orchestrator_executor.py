#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String
import requests
import json
import time
import re
import math
import subprocess
from datetime import datetime

class Phase3PromptOrchestratorExecutor(Node):
    """ROS2 Task Executor with PromptOrchestrator Chain-of-Thought Integration"""
    
    def __init__(self):
        super().__init__('phase3_prompt_orchestrator_executor')
        
        self.load_azure_config()
        
        # Publishers
        self.nav_pub = self.create_publisher(PoseStamped, '/move_base_simple/goal', 10)
        self.excavator_pub = self.create_publisher(Twist, '/excavator/cmd_vel', 10)
        self.level_pub = self.create_publisher(String, '/robot/level', 10)
        self.install_pub = self.create_publisher(String, '/robot/install', 10)
        self.finish_pub = self.create_publisher(String, '/robot/finish', 10)
        self.status_pub = self.create_publisher(String, '/digital_twin/status', 10)
        self.reasoning_pub = self.create_publisher(String, '/llm/reasoning_steps', 10)
        
        self.digital_twin_spaces = self.config.get("digital_twin_building", {}).get("spaces", {})
        
        # Robot state management
        self.robots = {
            "azure_construction_robot_01": {
                "current_x": self.config["world_settings"]["robot1_start"]["x"],
                "current_y": self.config["world_settings"]["robot1_start"]["y"],
                "current_z": self.config["world_settings"]["robot1_start"]["z"],
                "current_space": "bedroom_01",
                "floor": 0,
                "azure_targets": [],
                "current_target": 0,
                "status": "active",
                "specialization": "ground_floor_construction",
                "work_spaces": ["bedroom_01", "bedroom_02", "kitchen", "bathroom"],
                "current_reasoning_step": None,
                "tasks_from_llm": []
            },
            "azure_construction_robot_02": {
                "current_x": self.config["world_settings"]["robot2_start"]["x"],
                "current_y": self.config["world_settings"]["robot2_start"]["y"],
                "current_z": self.config["world_settings"]["robot2_start"]["z"],
                "current_space": "lounge",
                "floor": 1,
                "azure_targets": [],
                "current_target": 0,
                "status": "active",
                "specialization": "upper_floor_construction",
                "work_spaces": ["lounge"],
                "current_reasoning_step": None,
                "tasks_from_llm": []
            }
        }
        
        # Chain-of-Thought tracking
        self.current_orchestration = None
        self.reasoning_history = []
        self.task_counter = 0
        self.total_movements = 0
        
        # Timers for orchestration and execution
        self.orchestration_timer = self.create_timer(45.0, self.fetch_prompt_orchestration)
        self.execution_timer = self.create_timer(2.0, self.execute_llm_driven_movements)
        self.status_timer = self.create_timer(10.0, self.publish_digital_twin_status)
        
        self.get_logger().info('🎯 Phase 3 PromptOrchestrator-Driven Executor Started!')
        self.get_logger().info(f'📡 Connected to: {self.base_url}')
        self.get_logger().info(f'🧠 Chain-of-Thought reasoning: ENABLED')
        self.get_logger().info(f'🏗️ IFC Digital Twin Building: {len(self.digital_twin_spaces)} spaces')

    def load_azure_config(self):
        """Load Azure configuration with PromptOrchestrator settings"""
        try:
            with open('azure_config_advanced.json', 'r') as f:
                self.config = json.load(f)
            
            self.base_url = self.config["base_url"]
            self.function_code = self.config["function_code"]
            self.building_id = self.config["building_id"]
            self.orchestration_settings = self.config.get("orchestration_settings", {})
            
            self.get_logger().info('✅ Azure configuration loaded with PromptOrchestrator settings')
            self.get_logger().info(f'🧠 Complexity: {self.orchestration_settings.get("complexity", "detailed")}')
            self.get_logger().info(f'🔄 Reasoning depth: {self.orchestration_settings.get("reasoning_depth", 3)}')
            
        except Exception as e:
            self.get_logger().error(f'❌ Config error: {str(e)}')
            
    def fetch_prompt_orchestration(self):
        """Fetch Chain-of-Thought task orchestration from PromptOrchestrator"""
        try:
            self.get_logger().info('🧠 Fetching Chain-of-Thought orchestration from PromptOrchestrator...')
            
            # Call PromptOrchestrator for detailed reasoning
            complexity = self.orchestration_settings.get("complexity", "detailed")
            orchestrator_url = f"{self.base_url}{self.config['endpoints']['prompt_orchestrator']}"
            orchestrator_url += f"?building_id={self.building_id}&complexity={complexity}&code={self.function_code}"
            
            orchestration_response = requests.get(orchestrator_url, timeout=120)
            
            if orchestration_response.status_code == 200:
                orchestration_data = orchestration_response.json()
                self.get_logger().info('✅ Received Chain-of-Thought orchestration with reasoning steps')
                
                # Extract and process reasoning steps
                self.process_reasoning_steps(orchestration_data)
                
                # Extract final tasks from orchestration
                final_tasks = orchestration_data.get("orchestration_result", {}).get("final_tasks", {})
                
                # Convert to ROS2 commands using TaskOutputGenerator
                ros2_url = f"{self.base_url}{self.config['endpoints']['task_output_generator']}"
                ros2_url += f"?format=json&robot_type=construction_robot&code={self.function_code}"
                
                ros2_response = requests.post(ros2_url, json=final_tasks, timeout=15)
                
                if ros2_response.status_code == 200:
                    commands = ros2_response.json()
                    self.get_logger().info('✅ Converted LLM reasoning to ROS2 commands')
                    
                    # Process commands with reasoning context
                    self.process_llm_commands_with_reasoning(commands, orchestration_data)
                    self.publish_ros2_commands_with_reasoning(commands)
                    self.task_counter += 1
                    
        except Exception as e:
            self.get_logger().error(f'❌ PromptOrchestrator fetch error: {str(e)}')

    def process_reasoning_steps(self, orchestration_data):
        """Process and publish Chain-of-Thought reasoning steps"""
        try:
            reasoning_steps = orchestration_data.get("orchestration_result", {}).get("reasoning_steps", [])
            
            if reasoning_steps:
                self.current_orchestration = orchestration_data
                self.reasoning_history.append({
                    "timestamp": datetime.now().isoformat(),
                    "steps": reasoning_steps,
                    "building_id": self.building_id
                })
                
                # Publish reasoning steps for monitoring
                for step in reasoning_steps:
                    step_msg = String()
                    step_data = {
                        "step_number": step.get("step", 0),
                        "title": step.get("title", ""),
                        "reasoning": step.get("reasoning", "")[:500]  # Truncate for message size
                    }
                    step_msg.data = json.dumps(step_data)
                    self.reasoning_pub.publish(step_msg)
                    
                    self.get_logger().info(f'🧠 Reasoning Step {step.get("step", 0)}: {step.get("title", "")}')
                
                # Assign reasoning context to robots
                for robot_name in self.robots:
                    self.robots[robot_name]["current_reasoning_step"] = reasoning_steps[0] if reasoning_steps else None
                    
        except Exception as e:
            self.get_logger().error(f'❌ Error processing reasoning steps: {str(e)}')

    def process_llm_commands_with_reasoning(self, commands, orchestration_data):
        """Process LLM commands with Chain-of-Thought context"""
        try:
            # Extract tasks from ROS2 commands
            ros2_tasks = commands.get("ros2_commands", {}).get("tasks", [])
            reasoning_steps = orchestration_data.get("orchestration_result", {}).get("reasoning_steps", [])
            
            # Extract coordinates and actions with reasoning context
            coordinates_with_context = []
            
            for task in ros2_tasks:
                task_id = task.get("task_id", "")
                description = task.get("description", "")
                
                # Find corresponding reasoning for this task
                reasoning_context = None
                for step in reasoning_steps:
                    if any(keyword in step.get("reasoning", "").lower() 
                          for keyword in description.lower().split()):
                        reasoning_context = step
                        break
                
                # Extract movement commands from task
                for command in task.get("ros2_commands", []):
                    if command.get("command_type") == "navigation":
                        params = command.get("parameters", {})
                        target = params.get("target_position", {})
                        
                        x = target.get("x", 0.0)
                        y = target.get("y", 0.0)
                        z = target.get("z", 0.0)
                        
                        # Determine space based on coordinates
                        space_info = self.determine_building_space(x, y, z)
                        
                        coordinates_with_context.append({
                            "x": x, "y": y, "z": z,
                            "space": space_info["space"],
                            "floor": space_info["floor"],
                            "space_type": space_info["type"],
                            "task_id": task_id,
                            "task_description": description,
                            "reasoning_context": reasoning_context,
                            "safety_checks": task.get("safety_checks", [])
                        })
            
            # If no specific coordinates, generate based on reasoning
            if not coordinates_with_context:
                coordinates_with_context = self.generate_coordinates_from_reasoning(reasoning_steps)
            
            # Distribute tasks among robots based on floor and reasoning
            self.distribute_tasks_with_reasoning(coordinates_with_context)
            
        except Exception as e:
            self.get_logger().error(f'❌ Error processing LLM commands: {str(e)}')

    def generate_coordinates_from_reasoning(self, reasoning_steps):
        """Generate movement coordinates based on reasoning steps"""
        coordinates = []
        
        # Analyze reasoning to determine work areas
        for step in reasoning_steps:
            reasoning_text = step.get("reasoning", "").lower()
            
            # Site analysis step - survey entire building
            if "site" in reasoning_text or "survey" in reasoning_text:
                # Add survey points for each space
                for space_name, space_data in self.digital_twin_spaces.items():
                    pos = space_data.get("position", {})
                    coordinates.append({
                        "x": pos.get("x", 0),
                        "y": pos.get("y", 0),
                        "z": pos.get("z", 0) + 0.5,
                        "space": space_name,
                        "floor": space_data.get("floor", 0),
                        "space_type": space_data.get("type", "general"),
                        "task_description": f"Site survey of {space_name}",
                        "reasoning_context": step
                    })
            
            # Foundation work - ground floor focus
            elif "foundation" in reasoning_text or "structural" in reasoning_text:
                ground_spaces = ["bedroom_01", "bedroom_02", "kitchen", "bathroom"]
                for space_name in ground_spaces:
                    if space_name in self.digital_twin_spaces:
                        space_data = self.digital_twin_spaces[space_name]
                        pos = space_data.get("position", {})
                        coordinates.append({
                            "x": pos.get("x", 0),
                            "y": pos.get("y", 0),
                            "z": 0.5,
                            "space": space_name,
                            "floor": 0,
                            "space_type": space_data.get("type", "general"),
                            "task_description": f"Foundation work in {space_name}",
                            "reasoning_context": step
                        })
            
            # Upper floor construction
            elif "upper" in reasoning_text or "floor" in reasoning_text:
                if "lounge" in self.digital_twin_spaces:
                    space_data = self.digital_twin_spaces["lounge"]
                    pos = space_data.get("position", {})
                    coordinates.append({
                        "x": pos.get("x", 3.0),
                        "y": pos.get("y", 1.0),
                        "z": 4.7,
                        "space": "lounge",
                        "floor": 1,
                        "space_type": space_data.get("type", "commercial"),
                        "task_description": "Upper floor construction in lounge",
                        "reasoning_context": step
                    })
        
        return coordinates if coordinates else self.generate_default_coordinates()

    def generate_default_coordinates(self):
        """Generate default coordinates if reasoning doesn't provide specific locations"""
        return [
            {"x": -3.5, "y": -2.0, "z": 0.5, "space": "bedroom_01", "floor": 0, 
             "space_type": "residential", "task_description": "General construction"},
            {"x": 2.0, "y": -2.5, "z": 0.5, "space": "kitchen", "floor": 0,
             "space_type": "utility", "task_description": "Utility installation"},
            {"x": 3.0, "y": 1.0, "z": 4.7, "space": "lounge", "floor": 1,
             "space_type": "commercial", "task_description": "Upper floor work"}
        ]

    def distribute_tasks_with_reasoning(self, coordinates_with_context):
        """Distribute tasks to robots based on reasoning and floor assignment"""
        ground_floor_tasks = []
        upper_floor_tasks = []
        
        for coord in coordinates_with_context:
            if coord.get("floor", 0) == 0:
                ground_floor_tasks.append(coord)
            else:
                upper_floor_tasks.append(coord)
        
        # Assign tasks to robots with reasoning context
        self.robots["azure_construction_robot_01"]["azure_targets"] = ground_floor_tasks
        self.robots["azure_construction_robot_01"]["tasks_from_llm"] = ground_floor_tasks
        
        self.robots["azure_construction_robot_02"]["azure_targets"] = upper_floor_tasks
        self.robots["azure_construction_robot_02"]["tasks_from_llm"] = upper_floor_tasks
        
        self.get_logger().info(f'🎯 Task distribution with reasoning:')
        self.get_logger().info(f'  🔵 Ground Floor Robot: {len(ground_floor_tasks)} tasks')
        self.get_logger().info(f'  🔴 Upper Floor Robot: {len(upper_floor_tasks)} tasks')

    def publish_ros2_commands_with_reasoning(self, commands):
        """Publish ROS2 commands with Chain-of-Thought context"""
        try:
            tasks = commands.get("ros2_commands", {}).get("tasks", [])
            
            for task in tasks:
                task_id = task.get("task_id", "Unknown")
                description = task.get("description", "Construction task")
                safety_checks = task.get("safety_checks", [])
                
                # Create enhanced task message with reasoning
                task_msg = String()
                task_data = {
                    "task_id": task_id,
                    "description": description,
                    "building": self.config['digital_twin_building']['structure_type'],
                    "chain_of_thought": True,
                    "reasoning_steps": len(self.reasoning_history[-1]["steps"]) if self.reasoning_history else 0,
                    "safety_checks": safety_checks,
                    "timestamp": datetime.now().isoformat()
                }
                task_msg.data = json.dumps(task_data)
                
                # Determine task type and publish to appropriate topic
                description_lower = description.lower()
                
                if any(word in description_lower for word in ["site", "prep", "foundation", "excavat"]):
                    self.level_pub.publish(task_msg)
                    self.get_logger().info(f'📤 Site preparation task with reasoning: {task_id}')
                elif any(word in description_lower for word in ["struct", "frame", "wall", "install"]):
                    self.install_pub.publish(task_msg)
                    self.get_logger().info(f'📤 Structural task with reasoning: {task_id}')
                elif any(word in description_lower for word in ["finish", "interior", "paint", "final"]):
                    self.finish_pub.publish(task_msg)
                    self.get_logger().info(f'📤 Finishing task with reasoning: {task_id}')
                else:
                    self.level_pub.publish(task_msg)
                    self.get_logger().info(f'📤 General task with reasoning: {task_id}')
                
        except Exception as e:
            self.get_logger().error(f'❌ ROS2 publish error: {str(e)}')

    def execute_llm_driven_movements(self):
        """Execute robot movements based on LLM reasoning"""
        try:
            for robot_name, robot in self.robots.items():
                if robot["tasks_from_llm"]:
                    self.move_robot_with_reasoning(robot_name, robot)
                else:
                    self.execute_default_patrol(robot_name, robot)
                    
        except Exception as e:
            self.get_logger().error(f'❌ Movement execution error: {str(e)}')

    def move_robot_with_reasoning(self, robot_name, robot):
        """Move robot based on LLM-generated tasks with reasoning context"""
        tasks = robot["tasks_from_llm"]
        target_idx = robot["current_target"]
        
        if target_idx < len(tasks):
            task = tasks[target_idx]
            
            new_x = task["x"]
            new_y = task["y"]
            new_z = task["z"]
            
            success = self.move_robot_gazebo(robot_name, new_x, new_y, new_z)
            
            if success:
                robot["current_x"] = new_x
                robot["current_y"] = new_y
                robot["current_z"] = new_z
                robot["current_space"] = task.get("space", "unknown")
                robot["floor"] = task.get("floor", 0)
                robot["current_target"] = target_idx + 1
                
                self.total_movements += 1
                
                # Log movement with reasoning context
                reasoning = task.get("reasoning_context", {})
                self.get_logger().info(
                    f'✅ {robot_name}: Moved to {task.get("space")} '
                    f'({new_x:.2f}, {new_y:.2f}) for {task.get("task_description", "task")}'
                )
                if reasoning:
                    self.get_logger().info(f'   🧠 Reasoning: {reasoning.get("title", "")}')
                
                self.publish_navigation_goal(new_x, new_y, new_z)
        else:
            # Reset to start of task list
            robot["current_target"] = 0

    def execute_default_patrol(self, robot_name, robot):
        """Execute default patrol pattern when no LLM tasks available"""
        step = robot.get("patrol_step", 0)
        
        if robot_name == "azure_construction_robot_01":
            angle = step * 0.3
            new_x = -3.5 + 2 * math.cos(angle)
            new_y = -2.0 + 1.5 * math.sin(angle)
            new_z = 0.5
            robot["current_space"] = "bedroom_01"
            robot["floor"] = 0
        else:
            angle = step * 0.12
            new_x = 4.0 + 1.5 * math.sin(angle)
            new_y = 1.5 + 1.0 * math.sin(2 * angle)
            new_z = 4.7
            robot["current_space"] = "lounge"
            robot["floor"] = 1
        
        success = self.move_robot_gazebo(robot_name, new_x, new_y, new_z)
        
        if success:
            robot["current_x"] = new_x
            robot["current_y"] = new_y
            robot["current_z"] = new_z
            robot["patrol_step"] = step + 1
            self.total_movements += 1
            self.publish_navigation_goal(new_x, new_y, new_z)

    def move_robot_gazebo(self, robot_name, x, y, z):
        """Move robot in Gazebo simulation"""
        try:
            cmd = [
                'gz', 'service', '-s', f'/world/{self.config["world_settings"]["world_name"]}/set_pose',
                '--reqtype', 'gz.msgs.Pose',
                '--reptype', 'gz.msgs.Boolean',
                '--timeout', '1000',
                '--req', 
                f'name: "{robot_name}" position: {{x: {x}, y: {y}, z: {z}}} orientation: {{x: 0, y: 0, z: 0, w: 1}}'
            ]
            
            result = subprocess.run(cmd, capture_output=True, timeout=3, text=True)
            return result.returncode == 0
            
        except Exception:
            return False

    def publish_navigation_goal(self, x, y, z):
        """Publish navigation goal to ROS2"""
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)
        msg.pose.orientation.w = 1.0
        
        self.nav_pub.publish(msg)

    def determine_building_space(self, x, y, z):
        """Determine which building space coordinates belong to"""
        if z > 3:
            return {"space": "lounge", "floor": 1, "type": "commercial"}
        else:
            if x < -4:
                return {"space": "bedroom_01", "floor": 0, "type": "residential"}
            elif x < 0 and y < -2:
                return {"space": "bedroom_02", "floor": 0, "type": "residential"}
            elif x > 0 and y < -1:
                return {"space": "kitchen", "floor": 0, "type": "utility"}
            else:
                return {"space": "bathroom", "floor": 0, "type": "utility"}

    def publish_digital_twin_status(self):
        """Publish comprehensive status including Chain-of-Thought reasoning"""
        try:
            status_data = {
                "timestamp": datetime.now().isoformat(),
                "building_type": self.config["digital_twin_building"]["structure_type"],
                "total_floors": self.config["digital_twin_building"]["total_floors"],
                "building_area": self.config["world_settings"]["building_area"],
                "total_spaces": len(self.digital_twin_spaces),
                "orchestration_mode": "PromptOrchestrator_ChainOfThought",
                "tasks_completed": self.task_counter,
                "total_movements": self.total_movements,
                "reasoning_history_count": len(self.reasoning_history),
                "current_reasoning_steps": len(self.reasoning_history[-1]["steps"]) if self.reasoning_history else 0,
                "robots": {}
            }
            
            for robot_name, robot in self.robots.items():
                status_data["robots"][robot_name] = {
                    "current_space": robot["current_space"],
                    "floor": robot["floor"],
                    "position": {
                        "x": robot["current_x"],
                        "y": robot["current_y"],
                        "z": robot["current_z"]
                    },
                    "status": robot["status"],
                    "specialization": robot["specialization"],
                    "llm_tasks_count": len(robot["tasks_from_llm"]),
                    "current_reasoning": robot["current_reasoning_step"]["title"] if robot["current_reasoning_step"] else None,
                    "work_spaces": robot["work_spaces"]
                }
            
            status_msg = String()
            status_msg.data = json.dumps(status_data)
            self.status_pub.publish(status_msg)
            
            # Save telemetry with reasoning
            with open('prompt_orchestrator_telemetry.json', 'w') as f:
                json.dump(status_data, f, indent=2)
            
            # Save reasoning history separately
            if self.reasoning_history:
                with open('chain_of_thought_history.json', 'w') as f:
                    json.dump(self.reasoning_history, f, indent=2)
            
        except Exception as e:
            self.get_logger().error(f'❌ Status publish error: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    executor = Phase3PromptOrchestratorExecutor()
    
    try:
        rclpy.spin(executor)
    except KeyboardInterrupt:
        pass
    finally:
        executor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
