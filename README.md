# assignment_2_2024

This package `assignment_2_2024` implements a robot simulation in ROS with multiple components, including:
- An action client that sends a target (x, y) to an action server, with real-time feedback on progress.
- A service node that provides the last sent target coordinates.
- A simulation environment for testing the system in Gazebo.

The nodes and services are designed to allow a robot to move to a specified target and provide updates on its position and velocity.

## Directory Structure

assignment_2_2024/ ├── action/ │ └── Planning.action ├── config/ │ ├── sim.rviz │ └── sim2.rviz ├── launch/ │ ├── assignment1.launch │ ├── assignment2.launch │ └── sim_w1.launch ├── msg/ │ └── RobotPosition.msg ├── scripts/ │ ├── bug_as.py │ ├── go_to_point_service.py │ ├── Node_A.py │ ├── Node_B.py │ └── wall_follow_service.py ├── urdf/ │ ├── robot2_laser.gazebo │ └── robot2_laser.xacro └── world/ └── assignment.world
