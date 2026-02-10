# -Vision-Based-Pick-Place-Gazebo-RViz-
# ROS 2 Jazzy Pick & Place Simulation (xArm7)

## 📌 Project Overview
This project demonstrates an autonomous **Pick and Place** operation using the **xArm7** robotic manipulator in **ROS 2 Jazzy**. 

It features a custom Python automation script that coordinates the robot's movements with a simulated "smart object" in RViz, showcasing:
* **Kinematics:** Joint-space trajectory planning.
* **Visualization:** Custom Rviz markers to simulate object interaction.
* **State Machine Logic:** A finite state machine (Approach -> Pick -> Lift -> Place) controlling the robot.

## 🚀 Features
* **Robot:** xArm7 (7 DoF) with Gripper end-effector.
* **Simulation:** RViz2 environment (No heavy Gazebo physics required).
* **Smart Object:** A dynamic Red Cube that detects gripper commands and attaches/detaches automatically.
* **Automation:** `auto_bot.py` script handles the full pick-and-place lifecycle without manual intervention.

## 🛠️ Prerequisites
* **OS:** Ubuntu 24.04 (Noble Numbat)
* **ROS Version:** ROS 2 Jazzy Jalisco
* **Dependencies:** * `xarm_description`
    * `rclpy`
    * `visualization_msgs`

## 📦 Installation
1.  **Clone the Repository:**
    ```bash
    cd ~/robot_ws/src
    git clone [https://github.com/YOUR_USERNAME/YOUR_REPO_NAME.git](https://github.com/YOUR_USERNAME/YOUR_REPO_NAME.git)
    ```

2.  **Build the Workspace:**
    ```bash
    cd ~/robot_ws
    colcon build
    source install/setup.bash
    ```

## 🎮 Usage Guide
To run the full simulation, you need **3 separate terminals**.

### **Terminal 1: Launch Robot & RViz**
Loads the xArm7 robot with the gripper visual.
```bash
source ~/robot_ws/install/setup.bash
ros2 launch xarm_description xarm7_rviz_display.launch.py add_gripper:=true


Terminal 2: Spawn the Object
Creates the Red Cube and listens for gripper interactions.

Bash
source ~/robot_ws/install/setup.bash
python3 ~/robot_ws/src/smart_object.py
Terminal 3: Run Automation
Starts the autonomous Pick & Place sequence.

Bash
source ~/robot_ws/install/setup.bash
python3 ~/robot_ws/src/auto_bot.py
