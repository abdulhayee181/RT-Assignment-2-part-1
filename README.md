# assignment_2_2024

This package `assignment_2_2024` implements a robot simulation in ROS with multiple components, including:
- An action client that sends a target (x, y) to an action server, with real-time feedback on progress.
- A service node that provides the last sent target coordinates.
- A simulation environment for testing the system in Gazebo.

The nodes and services are designed to allow a robot to move to a specified target and provide updates on its position and velocity.


### Explanation of Directories:

- **action/**: Contains the action definition file `Planning.action`, which is used to define the goal and feedback structure for the action server.
- **config/**: Holds configuration files for the simulation environment, such as `sim.rviz` and `sim2.rviz`, for setting up the RViz visualization.
- **launch/**: Contains ROS launch files to start the simulation, nodes, and services.
    - `assignment1.launch`: Launches the basic robot simulation with default configuration.
    - `assignment2.launch`: Launches the simulation along with the action client node, action server, and the relevant services.
    - `sim_w1.launch`: Initializes the simulation with a specific world and robot model.
- **msg/**: Contains the custom message file `RobotPosition.msg`, used to communicate the robot’s position and velocity.
- **scripts/**: Contains the Python scripts implementing the ROS nodes and services:
    - `bug_as.py`: Likely a script implementing the action server.
    - `go_to_point_service.py`: Implements the service node that returns the last sent target coordinates.
    - `Node_A.py`: Implements the action client that sends the target `(x, y)` to the action server and listens for feedback.
    - `Node_B.py`: Likely another node related to the system (you can provide details here if necessary).
    - `wall_follow_service.py`: Another service node, potentially related to robot behavior or navigation.
- **urdf/**: Contains the URDF description files for the robot model (`robot2_laser.gazebo`, `robot2_laser.xacro`), used for simulation.
- **world/**: Contains the Gazebo world file (`assignment.world`) for the simulation environment setup.

---

## Nodes and Functionality

### (a) Action Client Node

- **Node Name**: `Node_A.py`
- **Description**: 
    - This node acts as an action client in the ROS action framework.
    - The action client sends a target `(x, y)` to an action server to command the robot to move towards a specific location.
    - The node listens for feedback from the action server to monitor the robot's progress and status. 
    - It can also cancel the target if necessary.
    - Additionally, it subscribes to the `/odom` topic to obtain the robot's position and velocity, which are then published as a custom message (`RobotPosition.msg`).

### (b) Service Node

- **Node Name**: `go_to_point_service.py`
- **Description**:
    - This node implements a ROS service.
    - When called, the service returns the coordinates of the last target that was sent by the action client. 
    - This allows users to query the system for the current target coordinates.

---

## Custom Message

- **Message Name**: `RobotPosition.msg`
- **Description**: 
    - This custom message is used to publish the robot's position and velocity for monitoring purposes.
    - It contains the robot's position `(x, y)` and velocity `(vel_x, vel_z)` in the `geometry_msgs` format.

- **Message Structure**:

  ```plaintext
  float64 x
  float64 y
  float64 vel_x
  float64 vel_z

## How to Run the Code

Follow these steps to build and run the project:

### 1. Install Dependencies

Before running the simulation, ensure that you have the necessary ROS dependencies installed. Use the following commands to install the required packages:


sudo apt-get install ros-<distro>-actionlib ros-<distro>-rospy ros-<distro>-geometry-msgs ros-<distro>-std-msgs gazebo_ros_pkgs
Make sure to replace <distro> with your specific ROS distribution (e.g., melodic, noetic).

## 2. Build the Package
Navigate to the root directory of your ROS workspace (catkin_ws) and build the package using the following commands:

cd ~/catkin_ws
catkin_make
