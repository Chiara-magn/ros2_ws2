# **Research Track 2: Robot Navigation Assignment**

This project implements a basic ROS2 robot navigation stack with a textual UI. The system allows the user to send target poses to the robot, handles goal execution through an action-based architecture, performs proportional control for motion and final orientation alignment, supports preemption, and provides a UI for sending goals and STOP commands.

---

## **System Structure**

### Goal UI Interface Node
- Python interface: `navigation_ui.py`

### Components
- C++ Action Server that executes a `MoveToPose` action: `robot_nav_server.cpp`
- C++ Action Client: `move_to_pose_client.cpp`

---

## **Description**

### **UI Node (Python)**

This node provides a keyboard‑based interface to send the robot goal poses and stop the robot.

The user can:

- Send a navigation goal (`g`) by entering:
  - target **x** position
  - target **y** position
  - target **orientation**, choosing the unit (`d` for degrees, `r` for radians)

- Stop the robot immediately (`s`)  
  The STOP command cancels the current action goal and forces the robot to halt.

- Send multiple goals in sequence  
  New goals **preempt** the previous one (the server cancels the old goal safely).

- See feedback in real time  
  The UI prints the goal sent, while the server prints the robot’s current pose during execution.

The UI automatically converts degrees to radians.  
Commands are published to `/goal_pose`.

---

### **Action Client (C++)**

Receives goals from the UI Node and sends them to the Action Server through the `MoveToPose` action.

This component:

- Subscribes to `/goal_pose`
- Converts the message into a `MoveToPose` action goal
- Sends the goal to the Action Server
- Handles feedback and result callbacks

It acts as a bridge between the UI and the Action Server.

---

### **Action Server (C++)**

Implements the `MoveToPose` action.

Provided operations:

- Receives a goal pose (x, y, yaw)
- Moves the robot toward the goal using a proportional controller
- Aligns the robot to the final orientation
- Publishes feedback (current position)
- Supports preemption (a new goal cancels the previous one)
- Stops immediately when receiving a STOP message
- Limits angular velocity to avoid oscillations

The server subscribes to `/odom` and publishes velocity commands to `/cmd_vel`.

---

## **Actions**

### `MoveToPose.action`

```
geometry_msgs/PoseStamped target_pose
---
bool success
---
geometry_msgs/PoseStamped current_pose
```
---

## **Launch File**

A dedicated launch file is provided to automatically start action components and container:

- `move_to_pose_client.cpp`
- `robot_nav_server.cpp`

### **Run the system launch**
```bash
ros2 launch robot_nav start_components.launch.py
```
This launch file does **not** start the UI node, since it requires keyboard input and must be run manually.

---

## **How to Run the Full System**

Note: 
In RViz, set the Fixed Frame to map and enable the TF display to visualize the full TF2 tree

### **1. Build the workspace**

Terminal 1:
```bash
cd ~/Desktop/ros2_ws2
rm -rf build install log
colcon build 
source install/setup.bash
```

### **2. Start the simulation**

```bash
ros2 launch bme_gazebo_sensors spawn_robot_ex.launch.py
```

### **3. Start the navigation components (Action Server + Client)**

Terminal 2:
```bash
cd ~/Desktop/ros2_ws2
source install/setup.bash
ros2 launch robot_nav start_components.launch.py
```
You should see:
"NavigationServer active" in green
"MoveToPose client ready" in blue

### **4. Start the Python UI**

Terminal 3:
```bash
cd ~/Desktop/ros2_ws2/src/robot_nav/scripts
python3 navigation_ui.py
```
---

## **UI goal Commands**

Commands:

- `g` → send goal  
- `s` → STOP robot  

When sending a goal, the UI will ask for:
- x position
- y position
- orientation (degrees or radians)
- unit selection (d/r)

The UI validates input and converts degrees to radians automatically.

## Notes

- The UI must be started manually in a separate terminal because it requires interactive user input.
- New goals preempt the previous one.
- STOP cancels the current goal and halts the robot immediately.
