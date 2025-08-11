# 🏗️ Azure ROS2 Construction Dashboard

## 📌 Overview
The **Azure ROS2 Construction Dashboard** is a real-time monitoring dashboard that displays live commands from **Azure AI → ROS2 → Gazebo** for a simulated construction site.  
It visualizes robot activity such as:
- **Navigation** 🧭
- **Excavation** ⛏️
- **Leveling** 📏
- **Installation** 🔧
- **Finishing** ✨

This tool is part of **Phase 4** in the Azure BIM-to-Digital Twin & Robotics integration project.

---
![Uploading image.png…]()


## 🚀 Features
- Live ROS2 topic subscription and monitoring
- Real-time GUI updates with **Tkinter**
- Command statistics & robot activity logs
- Building coordinate reference display
- Automatic status updates for each robot task

---

## 📂 Project Structure
Phase 4/
├── azure_construction_visualizer.py # Main visualization script
├── requirements.txt # Python dependencies
└── README.md # Documentation

yaml
Copy
Edit

---

## 🛠️ Requirements

- **Python 3.8+**
- ROS2 (Foxy, Humble, or newer)
- Tkinter (for GUI)
- ROS2 message packages:
  - `geometry-msgs`
  - `std-msgs`

Install dependencies:
```bash
pip install -r requirements.txt
On Ubuntu/Debian systems, also run:

bash
Copy
Edit
sudo apt install python3-tk ros-${ROS_DISTRO}-geometry-msgs ros-${ROS_DISTRO}-std-msgs
▶️ Usage
1️⃣ Navigate to the Phase 4 folder
bash
Copy
Edit
cd Phase\ 4
2️⃣ Run the Visualizer
bash
Copy
Edit
python3 azure_construction_visualizer.py
Ensure ROS2 is sourced before running:

bash
Copy
Edit
source /opt/ros/${ROS_DISTRO}/setup.bash
📡 ROS2 Topics Monitored
Topic	Message Type	Purpose
/move_base_simple/goal	geometry_msgs/PoseStamped	Navigation target coordinates
/excavator/cmd_vel	geometry_msgs/Twist	Excavator movement control
/robot/level	std_msgs/String	Leveling task updates
/robot/install	std_msgs/String	Installation progress
/robot/finish	std_msgs/String	Finishing stage updates

📊 Example Output
less
Copy
Edit
[12:30:05] 🧭 NAVIGATION: Robot moving to (4.0, 1.5, 0.0) in 73m² building
[12:30:12] ⛏️ EXCAVATION: Linear: (0.20, 0.00, 0.00) - Azure AI foundation work
[12:30:18] 📏 LEVELING: Ground leveling - Azure TaskOutputGenerator command
📜 License
This project is part of the Azure BIM-Robot Pipeline and is licensed under the MIT License.
