cd ~/projects

# Create the Azure-ROS2 Construction Pipeline Visualizer
cat > azure_construction_visualizer.py << 'EOF'
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String
import tkinter as tk
from tkinter import ttk, scrolledtext
import threading
import time
import json
from datetime import datetime

class AzureConstructionVisualizer(Node):
    def __init__(self, gui_callback):
        super().__init__('azure_construction_visualizer')
        self.gui_callback = gui_callback
        
        # Subscribe to all ROS2 topics from your Azure bridge
        self.nav_sub = self.create_subscription(PoseStamped, '/move_base_simple/goal', self.navigation_callback, 10)
        self.excavator_sub = self.create_subscription(Twist, '/excavator/cmd_vel', self.excavator_callback, 10)
        self.level_sub = self.create_subscription(String, '/robot/level', self.level_callback, 10)
        self.install_sub = self.create_subscription(String, '/robot/install', self.install_callback, 10)
        self.finish_sub = self.create_subscription(String, '/robot/finish', self.finish_callback, 10)
        
        self.get_logger().info('🎬 Azure Construction Pipeline Visualizer - Monitoring all ROS2 topics!')

    def navigation_callback(self, msg):
        pos = msg.pose.position
        data = {
            'type': 'NAVIGATION',
            'position': f"({pos.x:.1f}, {pos.y:.1f}, {pos.z:.1f})",
            'timestamp': datetime.now().strftime("%H:%M:%S"),
            'details': f"Moving to building coordinates: X={pos.x:.1f}m, Y={pos.y:.1f}m, Z={pos.z:.1f}m"
        }
        self.gui_callback(data)

    def excavator_callback(self, msg):
        linear = msg.linear
        angular = msg.angular
        data = {
            'type': 'EXCAVATION',
            'velocity': f"Linear: ({linear.x:.2f}, {linear.y:.2f}, {linear.z:.2f})",
            'timestamp': datetime.now().strftime("%H:%M:%S"),
            'details': f"Excavator velocity - Linear: X={linear.x:.2f} Y={linear.y:.2f} Z={linear.z:.2f}, Angular: {angular.z:.2f}"
        }
        self.gui_callback(data)

    def level_callback(self, msg):
        data = {
            'type': 'LEVELING',
            'action': msg.data,
            'timestamp': datetime.now().strftime("%H:%M:%S"),
            'details': f"Robot leveling action: {msg.data}"
        }
        self.gui_callback(data)

    def install_callback(self, msg):
        data = {
            'type': 'INSTALLATION',
            'action': msg.data,
            'timestamp': datetime.now().strftime("%H:%M:%S"),
            'details': f"Robot installation: {msg.data}"
        }
        self.gui_callback(data)

    def finish_callback(self, msg):
        data = {
            'type': 'FINISHING',
            'action': msg.data,
            'timestamp': datetime.now().strftime("%H:%M:%S"),
            'details': f"Robot finishing work: {msg.data}"
        }
        self.gui_callback(data)

class AzureConstructionGUI:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("🏗️ Azure-ROS2 Construction Pipeline Visualizer")
        self.root.geometry("1200x800")
        self.root.configure(bg='#1e1e1e')
        
        # Statistics
        self.stats = {
            'NAVIGATION': 0,
            'EXCAVATION': 0,
            'LEVELING': 0,
            'INSTALLATION': 0,
            'FINISHING': 0,
            'total_commands': 0
        }
        
        self.setup_gui()
        self.start_time = time.time()

    def setup_gui(self):
        # Main title
        title_frame = tk.Frame(self.root, bg='#1e1e1e')
        title_frame.pack(fill='x', padx=10, pady=(10, 5))
        
        title_label = tk.Label(title_frame, 
                              text="🚀 Azure AI → ROS2 → Gazebo Construction Pipeline", 
                              font=('Arial', 16, 'bold'),
                              fg='#00ff00', bg='#1e1e1e')
        title_label.pack()
        
        subtitle_label = tk.Label(title_frame,
                                 text="Real-time monitoring of Azure TaskOutputGenerator controlling construction robots",
                                 font=('Arial', 10),
                                 fg='#888888', bg='#1e1e1e')
        subtitle_label.pack()

        # Building info frame
        building_frame = tk.LabelFrame(self.root, text="🏢 Building Information", 
                                      font=('Arial', 10, 'bold'),
                                      fg='#00aaff', bg='#1e1e1e')
        building_frame.pack(fill='x', padx=10, pady=5)
        
        building_info = tk.Label(building_frame,
                                text="Building: 73m² Mixed-Use | Floors: Ground Floor (BEDROOM 01, BEDROOM 02) + Roof (KITCHEN 102, BATHROOM 103, LOUNGE 104)",
                                font=('Arial', 9),
                                fg='#cccccc', bg='#1e1e1e')
        building_info.pack(padx=10, pady=5)

        # Statistics frame
        stats_frame = tk.LabelFrame(self.root, text="📊 Real-Time Command Statistics", 
                                   font=('Arial', 10, 'bold'),
                                   fg='#00aaff', bg='#1e1e1e')
        stats_frame.pack(fill='x', padx=10, pady=5)
        
        self.stats_labels = {}
        stats_grid = tk.Frame(stats_frame, bg='#1e1e1e')
        stats_grid.pack(fill='x', padx=10, pady=5)
        
        # Create statistics labels
        stats_items = [
            ('🧭', 'NAVIGATION', '#00ff00'),
            ('⛏️', 'EXCAVATION', '#ff6600'), 
            ('📏', 'LEVELING', '#ffff00'),
            ('🔧', 'INSTALLATION', '#00ffff'),
            ('✨', 'FINISHING', '#ff00ff'),
            ('📈', 'TOTAL', '#ffffff')
        ]
        
        for i, (icon, key, color) in enumerate(stats_items):
            col = i % 3
            row = i // 3
            
            label = tk.Label(stats_grid,
                           text=f"{icon} {key}: 0",
                           font=('Arial', 10, 'bold'),
                           fg=color, bg='#1e1e1e')
            label.grid(row=row, column=col, padx=20, pady=2, sticky='w')
            self.stats_labels[key] = label

        # Current status frame
        status_frame = tk.LabelFrame(self.root, text="🤖 Current Robot Status", 
                                    font=('Arial', 10, 'bold'),
                                    fg='#00aaff', bg='#1e1e1e')
        status_frame.pack(fill='x', padx=10, pady=5)
        
        self.status_label = tk.Label(status_frame,
                                    text="🔄 Waiting for Azure commands...",
                                    font=('Arial', 10),
                                    fg='#ffaa00', bg='#1e1e1e')
        self.status_label.pack(padx=10, pady=5)

        # Live activity log
        log_frame = tk.LabelFrame(self.root, text="📝 Live Azure → ROS2 Activity Log", 
                                 font=('Arial', 10, 'bold'),
                                 fg='#00aaff', bg='#1e1e1e')
        log_frame.pack(fill='both', expand=True, padx=10, pady=5)
        
        self.log_text = scrolledtext.ScrolledText(log_frame,
                                                 height=20,
                                                 font=('Consolas', 9),
                                                 bg='#2d2d2d',
                                                 fg='#ffffff',
                                                 insertbackground='#ffffff')
        self.log_text.pack(fill='both', expand=True, padx=5, pady=5)

        # Building coordinates reference
        coords_frame = tk.LabelFrame(self.root, text="📍 Building Coordinate Reference", 
                                    font=('Arial', 9, 'bold'),
                                    fg='#00aaff', bg='#1e1e1e')
        coords_frame.pack(fill='x', padx=10, pady=5)
        
        coords_text = """Ground Floor (Z=0.0): BEDROOM 01 (-3.5, -2.0), BEDROOM 02 (3.5, -2.0) | Roof Level (Z=4.2): KITCHEN (4.0, 1.5), LOUNGE (4.0, 1.5), BATHROOM (0.0, 3.0)"""
        coords_label = tk.Label(coords_frame,
                               text=coords_text,
                               font=('Arial', 8),
                               fg='#888888', bg='#1e1e1e')
        coords_label.pack(padx=10, pady=3)

        # Add initial log entry
        self.add_log_entry("🚀 Azure Construction Visualizer started - Monitoring ROS2 topics for Azure AI commands", "SYSTEM")

    def add_log_entry(self, message, command_type="INFO"):
        timestamp = datetime.now().strftime("%H:%M:%S")
        
        # Color coding based on command type
        colors = {
            'NAVIGATION': '#00ff00',
            'EXCAVATION': '#ff6600',
            'LEVELING': '#ffff00', 
            'INSTALLATION': '#00ffff',
            'FINISHING': '#ff00ff',
            'SYSTEM': '#888888',
            'INFO': '#cccccc'
        }
        
        color = colors.get(command_type, '#cccccc')
        log_entry = f"[{timestamp}] {message}\n"
        
        self.log_text.insert(tk.END, log_entry)
        self.log_text.see(tk.END)
        
        # Color the last line
        line_start = f"{self.log_text.index(tk.END)}-1l"
        line_end = f"{self.log_text.index(tk.END)}-1c"
        self.log_text.tag_add(command_type, line_start, line_end)
        self.log_text.tag_config(command_type, foreground=color)

    def update_stats(self, command_type):
        if command_type in self.stats:
            self.stats[command_type] += 1
            self.stats['total_commands'] += 1
            
            # Update labels
            for key, label in self.stats_labels.items():
                if key == 'TOTAL':
                    count = self.stats['total_commands']
                    icon = '📈'
                else:
                    count = self.stats[key]
                    icons = {'NAVIGATION': '🧭', 'EXCAVATION': '⛏️', 'LEVELING': '📏', 
                            'INSTALLATION': '🔧', 'FINISHING': '✨'}
                    icon = icons.get(key, '📊')
                
                label.config(text=f"{icon} {key}: {count}")

    def update_status(self, command_type, details):
        status_messages = {
            'NAVIGATION': f"🧭 Robot navigating to building position",
            'EXCAVATION': f"⛏️ Excavator active - site preparation",
            'LEVELING': f"📏 Robot performing leveling operations", 
            'INSTALLATION': f"🔧 Installing building components",
            'FINISHING': f"✨ Finishing work in progress"
        }
        
        message = status_messages.get(command_type, f"🤖 Processing {command_type}")
        self.status_label.config(text=message, fg='#00ff00')
        
        # Reset status after 5 seconds
        self.root.after(5000, lambda: self.status_label.config(text="🔄 Waiting for next Azure command...", fg='#ffaa00'))

    def handle_ros2_data(self, data):
        """Handle ROS2 data from the node"""
        command_type = data['type']
        timestamp = data['timestamp']
        details = data['details']
        
        # Create log message
        if command_type == 'NAVIGATION':
            position = data['position']
            log_msg = f"🧭 NAVIGATION: Robot moving to {position} in 73m² building"
        elif command_type == 'EXCAVATION':
            velocity = data['velocity']
            log_msg = f"⛏️ EXCAVATION: {velocity} - Azure AI foundation work"
        elif command_type == 'LEVELING':
            action = data['action']
            log_msg = f"📏 LEVELING: {action} - Azure TaskOutputGenerator command"
        elif command_type == 'INSTALLATION':
            action = data['action']
            log_msg = f"🔧 INSTALLATION: {action} - Building systems work"
        elif command_type == 'FINISHING':
            action = data['action']
            log_msg = f"✨ FINISHING: {action} - Interior completion work"
        else:
            log_msg = f"🤖 {command_type}: {details}"
        
        # Update GUI
        self.add_log_entry(log_msg, command_type)
        self.update_stats(command_type)
        self.update_status(command_type, details)

    def run(self):
        self.root.mainloop()

def main():
    rclpy.init()
    
    # Create GUI
    gui = AzureConstructionGUI()
    
    # Create ROS2 node with GUI callback
    node = AzureConstructionVisualizer(gui.handle_ros2_data)
    
    # Start ROS2 in separate thread
    def spin_ros2():
        try:
            rclpy.spin(node)
        except Exception as e:
            print(f"ROS2 Error: {e}")
    
    ros2_thread = threading.Thread(target=spin_ros2, daemon=True)
    ros2_thread.start()
    
    try:
        # Run GUI (blocks until window closed)
        gui.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
EOF

chmod +x azure_construction_visualizer.py

echo "🎬 Azure Construction Pipeline Visualizer created!"
echo ""
echo "🚀 Starting real-time Azure → ROS2 → Gazebo visualizer..."
echo "This will show live commands from your Azure TaskOutputGenerator!"
echo ""

python3 azure_construction_visualizer.py
