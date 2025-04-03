assignment_2_2024 - Robot Navigation Package
===========================================

.. toctree::
   :maxdepth: 2
   :caption: Contents:

   installation
   usage
   modules
   nodes
   services
   simulation
   api
   
Overview
--------

This ROS package implements a robot navigation system with:

- Action client-server architecture for goal-based navigation
- Obstacle avoidance using bug algorithm
- Wall following behavior
- Real-time position and velocity monitoring
- Gazebo simulation environment

Main Components:

- **Node A**: Action client for setting navigation goals
- **Node B**: Position and velocity monitoring
- **bug_as**: Action server implementing bug algorithm
- **go_to_point_service**: Basic navigation service
- **wall_follow_service**: Obstacle avoidance service