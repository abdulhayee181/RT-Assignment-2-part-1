Installation
============

Prerequisites
-------------

- ROS Noetic (or your distribution)
- Gazebo simulator
- Python 2.7/3.x (depending on ROS version)

Dependencies
------------

.. code-block:: bash

   sudo apt-get install ros-<distro>-actionlib \
       ros-<distro>-rospy \
       ros-<distro>-geometry-msgs \
       ros-<distro>-std-msgs \
       gazebo_ros_pkgs

Building the Package
--------------------

.. code-block:: bash

   cd ~/catkin_ws
   catkin_make
   source devel/setup.bash