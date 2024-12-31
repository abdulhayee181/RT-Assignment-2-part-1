#! /usr/bin/env python

import rospy
import actionlib
import actionlib.msg
import assignment_2_2024
import assignment_2_2024.msg

from std_srvs.srv import *
from geometry_msgs.msg import Point, Pose, Twist
from nav_msgs.msg import Odometry
from assignment_2_2024.msg import RobotPosition
from actionlib_msgs.msg import GoalStatus

def callback(msg):
    global pub
    
    # Get position and linear velocity from msg
    position = msg.pose.pose.position
    velocity = msg.twist.twist.linear
    
    # Create custom msg
    robot_info = RobotPosition()
    robot_info.x = position.x
    robot_info.y = position.y
    robot_info.vel_x = velocity.x
    robot_info.vel_y = velocity.y
    
    # Publish robot_info
    pub.publish(robot_info)

def feedback_callback(feedback):
    # This function is called to process feedback from the action server
    print(f"Feedback: Robot is at position x={feedback.current_pose.pose.position.x}, y={feedback.current_pose.pose.position.y}")

def client():
    # Creates the action client
    client = actionlib.SimpleActionClient('/reaching_goal', assignment_2_2024.msg.PlanningAction)
    
    # Wait for the server to be ready
    client.wait_for_server()
    
    print("Welcome to the Robot Control Interface \n")
    
    while not rospy.is_shutdown():
    
        # User interface
        print("Insert the desired position you want to reach \n")
        
        # Check whether the input is a number or not
        try:
            x = float(input("x: "))
            y = float(input("y: "))
            
            print("\n")
            
            # Check the inserted coordinates, such that the robot doesn't get stuck on a wall
            if -9.0 <= x <= 9.0 and -9.0 <= y <= 9.0:
                
                # Set the goal position with the previously entered coordinates
                goal = assignment_2_2024.msg.PlanningGoal()
                goal.target_pose.pose.position.x = x
                goal.target_pose.pose.position.y = y
                
                # Send the goal to the action server with feedback
                client.send_goal(goal, feedback_cb=feedback_callback)
                
                print("The goal coordinates have been successfully set! \n")
                
                cancel = input("Enter 'c' to cancel the goal, or press 'enter' to set the next goal: \n")
                
                if cancel == 'c':
                    # Cancel goal
                    client.cancel_goal()
                    print("The goal was successfully cancelled! \n") 
                
                # Wait for result and check the status
                client.wait_for_result()
                state = client.get_state()
                
                if state == GoalStatus.SUCCEEDED:
                    print("The robot has successfully reached the goal! \n")
                elif state == GoalStatus.PREEMPTED:
                    print("The goal was preempted (cancelled by user) \n")
                else:
                    print(f"Goal failed with status: {state} \n")
                
            else:
                print("Error!! The inserted values are out of bound, retry! \n")
        
        except ValueError:
            print("Error!! The input must be a number, retry! \n")    

def main():
    """
    Main function
    
    This function initializes the publisher and the subscriber and then calls the function client() 
    """
    
    global pub
    
    # Initialize NodeA
    rospy.init_node("NodeA")
    
    # Custom msg publisher
    pub = rospy.Publisher("/robot_info", RobotPosition, queue_size=1)
    
    # Subscriber to /odom, get position and speed of the robot
    sub_odom = rospy.Subscriber('/odom', Odometry, callback)
    
    # Start client service
    client()

if __name__ == "__main__":
    main()  
