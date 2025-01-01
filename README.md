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

### 1. **Install Dependencies**
- **Description**:  
    Ensure that the required ROS dependencies are installed for running the simulation and nodes.  
    Use the following command to install the necessary packages:

    ```bash
    sudo apt-get install ros-<distro>-actionlib ros-<distro>-rospy ros-<distro>-geometry-msgs ros-<distro>-std-msgs gazebo_ros_pkgs
    ```

    Replace `<distro>` with your specific ROS distribution (e.g., `melodic`, `noetic`).

### 2. **Build the Package**
- **Description**:  
    After installing the dependencies, build the package within your ROS workspace.  
    Navigate to the root directory of your workspace (`catkin_ws`) and build the package:

    ```bash
    cd ~/catkin_ws
    catkin_make
    ```

### 3. **Source the Workspace**
- **Description**:  
    Before running the nodes, source the workspace to allow ROS to recognize the built packages:

    ```bash
    source devel/setup.bash
    ```

### 4. **Run the Simulation and Nodes**
- **Description**:  
    You are now ready to launch the simulation and nodes.  
    Use the following command to start the system with the action client, action server, and relevant services:

    ```bash
    roslaunch assignment_2_2024 assignment2.launch
    ```

    This will launch the robot simulation along with the action client (`Node_A.py`), the action server, and the relevant services.

- **Alternative Launch File**:  
    To launch the simulation with a specific world and configuration, use:

    ```bash
    roslaunch assignment_2_2024 sim_w1.launch
    ```

### 5. **Test the Service**
- **Description**:  
    Once the system is running, you can test the service that returns the last target coordinates sent by the action client.  
    Use the following `rosservice` command:

    ```bash
    rosservice call /get_last_target
    ```

    This will return the coordinates of the last target sent by the action client.

### 6. **Visualize in RViz**
- **Description**:  
    You can visualize the robot's movement and status in RViz.  
    Launch the RViz configuration file provided in the `config/` directory to visualize the simulation:

    ```bash
    roslaunch assignment_2_2024 assignment1.launch
    ```

    This will open RViz with the default configuration to view the robot’s movement and simulation.

---

## Node and Functionality

### (a) **Action Client Node**
- **Node Name**: `Node_A.py`
- **Description**:  
    - This node implements the action client.
    - It allows the user to set a target `(x, y)` or cancel the target.
    - The node sends the target to the action server and listens for feedback on the robot’s progress.
    - It also publishes the robot's position and velocity as a custom message (`RobotPosition.msg`).

- **Feedback/Status**:  
    The action client listens to feedback from the action server to know when the target has been reached.

### (b) **Service Node**
- **Node Name**: `go_to_point_service.py`
- **Description**:  
    This service node responds with the coordinates of the last target sent by the user.  
    It provides the target coordinates to any other nodes that query it.

---

## Conclusion
This assignment involves creating a simulation in ROS that integrates an action client, action server, and service nodes. The robot can move to a target `(x, y)` and provide real-time updates on its status. The service node allows querying the last target sent to the robot.
