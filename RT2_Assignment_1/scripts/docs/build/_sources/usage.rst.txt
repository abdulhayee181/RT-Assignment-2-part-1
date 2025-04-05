Usage
=====

Launching the System
--------------------

Basic simulation:

.. code-block:: bash

   roslaunch assignment_2_2024 assignment1.launch

Full system with action client:

.. code-block:: bash

   roslaunch assignment_2_2024 assignment2.launch

With specific world:

.. code-block:: bash

   roslaunch assignment_2_2024 sim_w1.launch

Interacting with Nodes
----------------------

1. Set target coordinates through Node A's interface
2. Monitor robot position and velocity through Node B
3. Cancel goals using 'c' command in Node A
4. Enable/disable services as needed