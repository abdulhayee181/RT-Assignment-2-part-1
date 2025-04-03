Services
========

.. bug_as - Action Server
.. ----------------------

.. **File**: ``bug_as.py``

.. .. automodule:: bug_as
..    :members:
..    :undoc-members:
..    :show-inheritance:


.. .. code-block:: bash


.. Description:
.. ^^^^^^^^^^^^
.. - Implements bug algorithm for navigation
.. - Handles obstacle avoidance
.. - Provides state machine for navigation
.. - Publishes feedback during navigation

go_to_point_service
-------------------

Description:
^^^^^^^^^^^^
- Basic point-to-point navigation
- Handles orientation correction
- State machine with 3 states:
  - Fix yaw
  - Go straight
  - Done


**File**: ``go_to_point_service.py``

.. automodule:: go_to_point_service
   :members:
   :undoc-members:
   :show-inheritance:

.. code-block:: bash
    
    #! /usr/bin/env python

    # import ros stuff
    import rospy
    from sensor_msgs.msg import LaserScan
    from geometry_msgs.msg import Twist, Point
    from nav_msgs.msg import Odometry
    from tf import transformations
    from std_srvs.srv import *
    import time

    import math

    active_ = False

    # robot state variables
    position_ = Point()
    yaw_ = 0
    # machine state
    state_ = 0
    # goal
    desired_position_ = Point()
    desired_position_.x = rospy.get_param('des_pos_x')
    desired_position_.y = rospy.get_param('des_pos_y')
    desired_position_.z = 0
    # parameters
    yaw_precision_ = math.pi / 9  # +/- 20 degree allowed
    yaw_precision_2_ = math.pi / 90  # +/- 2 degree allowed
    dist_precision_ = 0.3

    kp_a = 3.0  # In ROS Noetic, it may be necessary to change the sign of this proportional controller
    kp_d = 0.2
    ub_a = 0.6
    lb_a = -0.5
    ub_d = 0.6

    # publishers
    pub = None

    # service callbacks


    def go_to_point_switch(req):
        global active_
        active_ = req.data
        res = SetBoolResponse()
        res.success = True
        res.message = 'Done!'
        return res

    # callbacks


    def clbk_odom(msg):
        global position_
        global yaw_

        # position
        position_ = msg.pose.pose.position

        # yaw
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w)
        euler = transformations.euler_from_quaternion(quaternion)
        yaw_ = euler[2]


    def change_state(state):
        global state_
        state_ = state
        print ('State changed to [%s]' % state_)


    def normalize_angle(angle):
        if(math.fabs(angle) > math.pi):
            angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
        return angle


    def fix_yaw(des_pos):
        global yaw_, pub, yaw_precision_2_, state_
        desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
        err_yaw = normalize_angle(desired_yaw - yaw_)

        rospy.loginfo(err_yaw)

        twist_msg = Twist()
        if math.fabs(err_yaw) > yaw_precision_2_:
            twist_msg.angular.z = kp_a*err_yaw
            if twist_msg.angular.z > ub_a:
                twist_msg.angular.z = ub_a
            elif twist_msg.angular.z < lb_a:
                twist_msg.angular.z = lb_a

        pub.publish(twist_msg)

        # state change conditions
        if math.fabs(err_yaw) <= yaw_precision_2_:
            print ('Yaw error: [%s]' % err_yaw)
            change_state(1)


    def go_straight_ahead(des_pos):
        global yaw_, pub, yaw_precision_, state_
        desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
        err_yaw = desired_yaw - yaw_
        err_pos = math.sqrt(pow(des_pos.y - position_.y, 2) +
                            pow(des_pos.x - position_.x, 2))

        if err_pos > dist_precision_:
            twist_msg = Twist()
            twist_msg.linear.x = kp_d*(err_pos)
            if twist_msg.linear.x > ub_d:
                twist_msg.linear.x = ub_d

            twist_msg.angular.z = kp_a*err_yaw
            pub.publish(twist_msg)
        else:
            print ('Position error: [%s]' % err_pos)
            change_state(2)

        # state change conditions
        if math.fabs(err_yaw) > yaw_precision_:
            print ('Yaw error: [%s]' % err_yaw)
            change_state(0)


    def done():
        twist_msg = Twist()
        twist_msg.linear.x = 0
        twist_msg.angular.z = 0
        pub.publish(twist_msg)
                    

    def main():
        global pub, active_

        rospy.init_node('go_to_point')

        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)

        srv = rospy.Service('go_to_point_switch', SetBool, go_to_point_switch)

        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if not active_:
                continue
            else:
                desired_position_.x = rospy.get_param('des_pos_x')
                desired_position_.y = rospy.get_param('des_pos_y')
                if state_ == 0:
                    fix_yaw(desired_position_)
                elif state_ == 1:
                    go_straight_ahead(desired_position_)
                elif state_ == 2:
                    done()
                else:
                    rospy.logerr('Unknown state!')

            rate.sleep()


    if __name__ == '__main__':
        main()




wall_follow_service
-------------------

Description:
^^^^^^^^^^^^
- Implements wall following behavior
- 3 operational states:
  - Find wall
  - Turn left
  - Follow wall
- Handles 8 different obstacle scenarios

**File**: ``wall_follow_service.py``

.. automodule:: wall_follow_service
   :members:
   :undoc-members:
   :show-inheritance:

.. code-block:: bash

    #! /usr/bin/env python

    import rospy
    from sensor_msgs.msg import LaserScan
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from tf import transformations
    from std_srvs.srv import *

    import math

    active_ = False

    pub_ = None
    regions_ = {
        'right': 0,
        'fright': 0,
        'front': 0,
        'fleft': 0,
        'left': 0,
    }
    state_ = 0
    state_dict_ = {
        0: 'find the wall',
        1: 'turn left',
        2: 'follow the wall',
    }


    def wall_follower_switch(req):
        global active_
        active_ = req.data
        res = SetBoolResponse()
        res.success = True
        res.message = 'Done!'
        return res


    def clbk_laser(msg):
        global regions_
        regions_ = {
            'right':  min(min(msg.ranges[0:143]), 10),
            'fright': min(min(msg.ranges[144:287]), 10),
            'front':  min(min(msg.ranges[288:431]), 10),
            'fleft':  min(min(msg.ranges[432:575]), 10),
            'left':   min(min(msg.ranges[576:713]), 10),
        }

        take_action()


    def change_state(state):
        global state_, state_dict_
        if state is not state_:
            print ('Wall follower - [%s] - %s' % (state, state_dict_[state]))
            state_ = state


    def take_action():
        global regions_
        regions = regions_
        msg = Twist()
        linear_x = 0
        angular_z = 0
        state_description = ''

        d0 = 1
        d = 1.5

        if regions['front'] > d0 and regions['fleft'] > d and regions['fright'] > d:
            state_description = 'case 1 - nothing'
            change_state(0)
        elif regions['front'] < d0 and regions['fleft'] > d and regions['fright'] > d:
            state_description = 'case 2 - front'
            change_state(1)
        elif regions['front'] > d0 and regions['fleft'] > d and regions['fright'] < d:
            state_description = 'case 3 - fright'
            change_state(2)
        elif regions['front'] > d0 and regions['fleft'] < d and regions['fright'] > d:
            state_description = 'case 4 - fleft'
            change_state(0)
        elif regions['front'] < d0 and regions['fleft'] > d and regions['fright'] < d:
            state_description = 'case 5 - front and fright'
            change_state(1)
        elif regions['front'] < d0 and regions['fleft'] < d and regions['fright'] > d:
            state_description = 'case 6 - front and fleft'
            change_state(1)
        elif regions['front'] < d0 and regions['fleft'] < d and regions['fright'] < d:
            state_description = 'case 7 - front and fleft and fright'
            change_state(1)
        elif regions['front'] > d0 and regions['fleft'] < d and regions['fright'] < d:
            state_description = 'case 8 - fleft and fright'
            change_state(0)
        else:
            state_description = 'unknown case'
            rospy.loginfo(regions)


    def find_wall():
        msg = Twist()
        msg.linear.x = 0.2
        msg.angular.z = -0.3
        return msg


    def turn_left():
        msg = Twist()
        msg.angular.z = 0.3
        return msg


    def follow_the_wall():
        global regions_

        msg = Twist()
        msg.linear.x = 0.5
        return msg


    def main():
        global pub_, active_

        rospy.init_node('reading_laser')

        pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        sub = rospy.Subscriber('/scan', LaserScan, clbk_laser)

        srv = rospy.Service('wall_follower_switch', SetBool, wall_follower_switch)

        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if not active_:
                rate.sleep()
                continue
            else:
                msg = Twist()
                if state_ == 0:
                    msg = find_wall()
                elif state_ == 1:
                    msg = turn_left()
                elif state_ == 2:
                    msg = follow_the_wall()
                else:
                    rospy.logerr('Unknown state!')

                pub_.publish(msg)

            rate.sleep()


    if __name__ == '__main__':
        main()
