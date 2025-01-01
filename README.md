# assignment_2_2024

This package `assignment_2_2024` implements a robot simulation in ROS with multiple components, including:
- An action client that sends a target (x, y) to an action server, with real-time feedback on progress.
- A service node that provides the last sent target coordinates.
- A simulation environment for testing the system in Gazebo.

The nodes and services are designed to allow a robot to move to a specified target and provide updates on its position and velocity.

## Directory Structure

