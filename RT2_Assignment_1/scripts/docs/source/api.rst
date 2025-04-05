API Reference
=============

Custom Messages
---------------

RobotPosition.msg
^^^^^^^^^^^^^^^^^

.. code-block:: rosmsg

   float64 x
   float64 y
   float64 vel_x
   float64 vel_y

Planning.action
^^^^^^^^^^^^^^^

.. code-block:: rosmsg

   # Goal definition
   geometry_msgs/Pose target_pose
   ---
   # Result definition
   geometry_msgs/Pose final_pose
   ---
   # Feedback definition
   geometry_msgs/Pose current_pose
   string stat

Services
---------

/go_to_point_switch (std_srvs/SetBool)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
- Enable/disable go_to_point service

/wall_follower_switch (std_srvs/SetBool)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
- Enable/disable wall follower service