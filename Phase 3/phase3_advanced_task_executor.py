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

class Phase3AdvancedTaskExecutor(Node):
    def __init__(self):
        super().__init__('phase3_advanced_task_executor')
        
        self.load_azure_config()
        
        self.nav_pub = self.create_publisher(PoseStamped, '/move_base_simple/goal', 10)
        self.excavator_pub = self.create_publisher(Twist, '/excavator/cmd_vel', 10)
        self.level_pub = self.create_publisher(String, '/robot/level', 10)
        self.install_pub = self.create_publisher(String, '/robot/install', 10)
        self.finish_pub = self.create_publisher(String, '/robot/finish', 10)
        self.status_pub = self.create_publisher(String, '/digital_twin/status', 10)
        
        self.digital_twin_spaces = self.config.get("digital_twin_building", {}).get("spaces", {})
        
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
                "work_spaces": ["bedroom_01", "bedroom_02", "kitchen", "bathroom"]
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
                "work_spaces": ["lounge"]
            }
        }
        
        self.task_counter = 0
        self.total_movements = 0
        
        self.azure_timer = self.create_timer(30.0, self.fetch_azure_tasks)
        self.execution_timer = self.create_timer(2.0, self.execute_movements)
        self.status_timer = self.create_timer(10.0, self.publish_digital_twin_status)
        
        self.get_logger().info('🎯 Phase 3 IFC-Aligned Task Executor Started!')
        self.get_logger().info(f'📡 Connected to: {self.base_url}')
        self.get_logger().info(f'🏗️ IFC Digital Twin Building: {len(self.digital_twin_spaces)} spaces matching IFC model')

    def load_azure_config(self):
        try:
            with open('azure_config_advanced.json', 'r') as f:
                self.config = json.load(f)
            
            self.base_url = self.config["base_url"]
            self.function_code = self.config["function_code"]
            self.building_id = self.config["building_id"]
            
            self.get_logger().info('✅ IFC-aligned Azure configuration loaded')
            
        except Exception as e:
            self.get_logger().error(f'❌ Config error: {str(e)}')
            
    def fetch_azure_tasks(self):
        try:
            self.get_logger().info('📡 Fetching Azure construction tasks for IFC building...')
            
            task_url = f"{self.base_url}{self.config['endpoints']['task_planner']}?building_id={self.building_id}&task_type=comprehensive_construction&code={self.function_code}"
            task_response = requests.get(task_url, timeout=15)
            
            if task_response.status_code == 200:
                task_data = task_response.json()
                self.get_logger().info('✅ Received Azure task plan with IFC building data')
                
                ros2_url = f"{self.base_url}{self.config['endpoints']['task_output_generator']}?format=json&robot_type=construction_robot&code={self.function_code}"
                ros2_response = requests.post(ros2_url, json=task_data.get("task_plan", {}), timeout=15)
                
                if ros2_response.status_code == 200:
                    commands = ros2_response.json()
                    self.get_logger().info('✅ Received Azure ROS2 commands')
                    
                    self.extract_coordinates_with_building_context(commands, task_data)
                    self.publish_ros2_commands_advanced(commands)
                    self.task_counter += 1
                    
        except Exception as e:
            self.get_logger().error(f'❌ Azure fetch error: {str(e)}')

    def extract_coordinates_with_building_context(self, azure_commands, task_data):
        combined_text = json.dumps(azure_commands) + " " + json.dumps(task_data)
        numbers = re.findall(r'-?\d+\.?\d*', combined_text)
        
        coordinates = []
        for i in range(0, len(numbers) - 2, 3):
            try:
                x, y, z = float(numbers[i]), float(numbers[i+1]), float(numbers[i+2])
                if (-10 <= x <= 10 and -6 <= y <= 6 and 0 <= z <= 8):
                    space_info = self.determine_building_space(x, y, z)
                    coordinates.append({
                        "x": x, "y": y, "z": z, 
                        "space": space_info["space"],
                        "floor": space_info["floor"],
                        "space_type": space_info["type"]
                    })
            except:
                continue
        
        unique_coords = []
        for coord in coordinates:
            if not any(abs(coord["x"] - uc["x"]) < 0.1 and 
                      abs(coord["y"] - uc["y"]) < 0.1 for uc in unique_coords):
                unique_coords.append(coord)
        
        if unique_coords:
            ground_floor_coords = [c for c in unique_coords if c["floor"] == 0]
            upper_floor_coords = [c for c in unique_coords if c["floor"] == 1]
            
            if not ground_floor_coords or not upper_floor_coords:
                half_point = len(unique_coords) // 2
                ground_floor_coords = unique_coords[:half_point]
                upper_floor_coords = unique_coords[half_point:]
            
            self.robots["azure_construction_robot_01"]["azure_targets"] = ground_floor_coords
            self.robots["azure_construction_robot_02"]["azure_targets"] = upper_floor_coords
            
            self.get_logger().info(f'🎯 Extracted {len(unique_coords)} coordinates for IFC spaces')
            self.get_logger().info(f'🔵 Ground Floor Robot: {len(ground_floor_coords)} targets')
            self.get_logger().info(f'🔴 Upper Floor Robot: {len(upper_floor_coords)} targets')

    def determine_building_space(self, x, y, z):
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

    def publish_ros2_commands_advanced(self, commands):
        try:
            tasks = commands.get("tasks", [])
            
            for task in tasks:
                task_id = task.get("task_id", "Unknown")
                description = task.get("description", "Construction task")
                
                task_msg = String()
                task_msg.data = f"IFC Digital Twin Task {task_id}: {description} | Building: {self.config['digital_twin_building']['structure_type']}"
                
                description_lower = description.lower()
                
                if "level" in description_lower or "prepare" in description_lower or "site" in description_lower:
                    self.level_pub.publish(task_msg)
                    self.get_logger().info(f'📤 ROS2: Site preparation task published for IFC building')
                elif "install" in description_lower or "structural" in description_lower:
                    self.install_pub.publish(task_msg)
                    self.get_logger().info(f'📤 ROS2: Installation task published with IFC space awareness')
                elif "finish" in description_lower or "interior" in description_lower:
                    self.finish_pub.publish(task_msg)
                    self.get_logger().info(f'📤 ROS2: Finishing task published for IFC building spaces')
                else:
                    self.level_pub.publish(task_msg)
                    self.get_logger().info(f'📤 ROS2: General construction task published')
                
        except Exception as e:
            self.get_logger().error(f'❌ ROS2 publish error: {str(e)}')

    def execute_movements(self):
        try:
            for robot_name, robot in self.robots.items():
                if robot["azure_targets"]:
                    self.move_to_azure_target_with_building_context(robot_name, robot)
                else:
                    self.execute_building_patrol(robot_name, robot)
                    
        except Exception as e:
            self.get_logger().error(f'❌ Movement error: {str(e)}')

    def move_to_azure_target_with_building_context(self, robot_name, robot):
        targets = robot["azure_targets"]
        target_idx = robot["current_target"]
        target = targets[target_idx % len(targets)]
        
        new_x = target["x"]
        new_y = target["y"]
        new_z = 4.7 if target.get("floor") == 1 else 0.5
        
        success = self.move_robot_gazebo(robot_name, new_x, new_y, new_z)
        
        if success:
            robot["current_x"] = new_x
            robot["current_y"] = new_y
            robot["current_z"] = new_z
            robot["current_space"] = target.get("space", "unknown")
            robot["floor"] = target.get("floor", 0)
            robot["current_target"] = (target_idx + 1) % len(targets)
            
            self.total_movements += 1
            
            self.get_logger().info(f'✅ {robot_name}: Azure target ({new_x:.2f}, {new_y:.2f}) in {robot["current_space"]}')
            self.publish_navigation_goal(new_x, new_y, new_z)

    def execute_building_patrol(self, robot_name, robot):
        step = robot.get("patrol_step", 0)
        work_spaces = robot["work_spaces"]
        
        if robot_name == "azure_construction_robot_01":
            angle = step * 0.3
            new_x = -3.5 + 2 * math.cos(angle)
            new_y = -2.0 + 1.5 * math.sin(angle)
            new_z = 0.5
            
            if work_spaces:
                current_space_idx = step % len(work_spaces)
                robot["current_space"] = work_spaces[current_space_idx]
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
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)
        msg.pose.orientation.w = 1.0
        
        self.nav_pub.publish(msg)

    def publish_digital_twin_status(self):
        try:
            status_data = {
                "timestamp": datetime.now().isoformat(),
                "building_type": self.config["digital_twin_building"]["structure_type"],
                "total_floors": self.config["digital_twin_building"]["total_floors"],
                "building_area": self.config["world_settings"]["building_area"],
                "total_spaces": len(self.digital_twin_spaces),
                "tasks_completed": self.task_counter,
                "total_movements": self.total_movements,
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
                    "azure_targets": len(robot["azure_targets"]),
                    "work_spaces": robot["work_spaces"]
                }
            
            status_msg = String()
            status_msg.data = json.dumps(status_data)
            self.status_pub.publish(status_msg)
            
            with open('azure_ifc_digital_twin_telemetry.json', 'w') as f:
                json.dump(status_data, f, indent=2)
            
        except Exception as e:
            self.get_logger().error(f'❌ Status publish error: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    executor = Phase3AdvancedTaskExecutor()
    
    try:
        rclpy.spin(executor)
    except KeyboardInterrupt:
        pass
    finally:
        executor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
