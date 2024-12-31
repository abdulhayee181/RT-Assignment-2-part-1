# #! /usr/bin/env python


# import rospy
# import actionlib
# import actionlib.msg
# # import assignment_2_2024
# import msg

# from std_srvs.srv import *
# from geometry_msgs.msg import Point, Pose, Twist
# from nav_msgs.msg import Odometry
# from msg import InfoMsg

# def callback(msg):
#     """
#     Callback function to publish position and velocity of the robot taken via the custom message InfoMsg based on the information retrieved from the topic /odom
    
#     Args: 
#         msg of type Odometry: contains the odometry information of the robot
#     """

#     global pub
    
#     # Get position and linear velocity from msg
#     position = msg.pose.pose.position
#     velocity = msg.twist.twist.linear
    
#     # Create custom message
#     robot_info = InfoMsg()
#     robot_info.x = position.x
#     robot_info.y = position.y
#     robot_info.velX = velocity.x
#     robot_info.velY = velocity.y
    
#     # Publish robot_info
#     pub.publish(robot_info)
    
# def client():
#     """
#     Function that implements the action client and gives the user the possibility to set/cancel goals.
    
#     This function initializes the action client and waits for an input by the user. If the input is acceptable,
#     a new goal is set by sending the inserted coordinates to the action server. The user can also cancel the goal.
#     """

#     # Creates the action client
#     client = actionlib.SimpleActionClient('/reaching_goal', msg.PlanningAction)
    
#     # Waits for the server to be ready
#     client.wait_for_server()

#     print("Welcome to the Robot Control Interface \n")

#     while not rospy.is_shutdown():

#         # User interface
#         print("Insert the desired position you want to reach \n")

#         try:
#             x = float(input("x: "))
#             y = float(input("y: "))
#             print("\n")

#             # Check if the coordinates are within bounds
#             if -9.0 <= x <= 9.0 and -9.0 <= y <= 9.0:
#                 goal = msg.PlanningGoal()
#                 goal.target_pose.pose.position.x = x
#                 goal.target_pose.pose.position.y = y

#                 # Send the goal to the action server
#                 client.send_goal(goal)

#                 # Wait for the result and get feedback
#                 while not client.get_state() == actionlib.GoalStatus.SUCCEEDED:
#                     feedback = client.get_feedback()
#                     if feedback:
#                         print(f"Current position: ({feedback.current_position.x}, {feedback.current_position.y}), "
#                               f"Current velocity: ({feedback.current_velocity.x}, {feedback.current_velocity.y})")

#                 print("The goal coordinates have been successfully set! \n")

#                 cancel = input("Enter 'c' to cancel the goal, or press 'enter' to set the next goal: \n")

#                 if cancel == 'c':
#                     # Cancel goal
#                     client.cancel_goal()
#                     print("The goal was successfully cancelled! \n")

#             else:
#                 print("Error!! The inserted values are out of bounds, retry! \n")

#         except ValueError:
#             print("Error!! The input must be a number, retry! \n")  

# def main():
#     """
#     Main function.
    
#     This function initializes the publisher and the subscriber, then calls the client() function.
#     """
    
#     global pub
    
#     # Initialize NodeA
#     rospy.init_node("robot_control_node")
    
#     # Custom msg publisher
#     pub = rospy.Publisher("/robot_info", InfoMsg, queue_size=1)
    
#     # Subscriber to /odom, get position and speed of the robot
#     sub_odom = rospy.Subscriber('/odom', Odometry, callback)
    
#     # Start client service
#     client()

# if __name__ == "__main__":
#     main()














































#!/usr/bin/env python

import rospy
import actionlib
from nav_msgs.msg import Odometry


from assignment_2_2024.msg import RobotPosition  # Custom message you will create
from assignment_2_2024.action import PlanningAction, MoveToGoalAction, MoveToGoalGoal  # Replace with actual action server message

class ActionClientNode:
    def __init__(self):
        rospy.init_node('action_client_node')  # Initialize the ROS node
        
        # Action client setup: connecting to the action server
        self.client = actionlib.SimpleActionClient('/move_to_goal', MoveToGoalAction)  
        rospy.loginfo("Waiting for action server to start...")
        self.client.wait_for_server()
        
        rospy.loginfo("Connected to action server.")
        
        # Publisher for robot's position and velocity
        self.pub = rospy.Publisher('/robot_position', RobotPosition, queue_size=10)
        
        # Subscriber to /odom to get position and velocity updates
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        
        # Store the robot's current position and velocity
        self.current_position = (0, 0)
        self.current_velocity = (0, 0)

    def odom_callback(self, msg):
        # Extract position and velocity from the Odometry message
        self.current_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.current_velocity = (msg.twist.twist.linear.x, msg.twist.twist.linear.z)

        # Publish the robot's current position and velocity as a custom message
        robot_msg = RobotPosition()
        robot_msg.x = self.current_position[0]
        robot_msg.y = self.current_position[1]
        robot_msg.vel_x = self.current_velocity[0]
        robot_msg.vel_z = self.current_velocity[1]
        self.pub.publish(robot_msg)

    def send_goal(self, x, y):
        goal = MoveToGoalGoal()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        self.client.send_goal(goal, feedback_cb=self.feedback_callback)

    def feedback_callback(self, feedback):
        rospy.loginfo(f"Current position: {feedback.current_position}")
    
    def wait_for_result(self):
        self.client.wait_for_result()  # Wait for the result
        return self.client.get_result()

if __name__ == "__main__":
    # Initialize the action client node
    client_node = ActionClientNode()

    # Example target position to send to the action server
    target_x = 2.0
    target_y = 3.0
    client_node.send_goal(target_x, target_y)
    
    # Wait for the action to finish and get the result
    result = client_node.wait_for_result()
    rospy.loginfo(f"Goal reached: {result}")













