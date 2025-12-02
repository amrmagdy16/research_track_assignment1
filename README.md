# Research Track I - Assignment 1: UI & Distance Check

**Author:** Amr Magdy Mohamed Elsayed Abdalla
**Email:** amr.m.mohamed13@gmail.com
**Package:** assignment1_rt

## ⚙️ Implementation Details

This package solves Assignment 1 by implementing two independent ROS2 Python nodes that communicate via standard ROS2 interfaces (Topics).

| Node Name | Role | Communication | Key Feature |
| :--- | :--- | :--- | :--- |
| `ui_node` | **Control/Interface (Node 1)** | Publishes `geometry_msgs/Twist` | Provides a simple textual interface (`input()`) to drive `turtle1` or `turtle2` for 1 second before automatically stopping the movement. |
| `distance_node` | **Monitor/Safety (Node 2)** | Subscribes to `turtlesim/Pose` for both turtles | Calculates Euclidean distance. Publishes the distance to the `/turtle_distance` topic (`std_msgs/Float32`). Stops the turtles if distance is below a threshold or if they reach the boundaries (x/y > 10.0 or < 1.0). |

## ▶️ How to Run the Code (4 Terminals)

Make sure you are sourced into your workspace (`source ~/rt1_assignment_ws/install/local_setup.bash`) in all terminals before running the nodes.

| Terminal | Command | Purpose |
| :--- | :--- | :--- |
| **Terminal 1** | `ros2 run turtlesim turtlesim_node` | Starts the simulation environment. |
| **Terminal 2** | `ros2 service call /spawn turtlesim/srv/Spawn "{x: 5.0, y: 5.0, theta: 0.0, name: 'turtle2'}"` | Creates the second turtle required for the assignment. |
| **Terminal 3** | `ros2 run assignment1_rt distance_node` | Starts the safety monitor (runs in the background). |
| **Terminal 4** | `ros2 run assignment1_rt ui_node` | Starts the control interface (where you enter commands). |
