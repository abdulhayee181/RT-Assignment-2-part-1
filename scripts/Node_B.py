#! /usr/bin/env python


import rospy
import math
from assignment_2_2024.msg import RobotPosition

def callback(msg):
    """
    Callback function that calculates the distance of the robot from the goal and the average speed 
    based on the values retrieved from the topic /robot_info.

    Args: 
        msg: custom msg of type InfoMsg containing the current coordinates and the velocity of the robot.
    """
    # Get the goal position from parameters (with default values if not set)
    desX = rospy.get_param("des_pos_x", 0.0)  # Default value: 0.0
    desY = rospy.get_param("des_pos_y", 0.0)  # Default value: 0.0
    
    # Get the robot's position and velocity from the message
    x = msg.x
    y = msg.y
    velX = msg.vel_x
    velY = msg.vel_y
    
    # Calculate the distance between the robot's current position and the goal
    distance = math.sqrt(pow(desX - x, 2) + pow(desY - y, 2))
    
    # Calculate the robot's speed (average speed based on velocity components)
    speed = math.sqrt(pow(velX, 2) + pow(velY, 2))
    
    # Print the information
    print("The distance from the goal position is: {:.2f}".format(distance))
    print("The robot's average speed is: {:.2f}".format(speed))
    print("\n")

def main():
    """
    Main function that initializes the subscriber, retrieves parameters,
    and sets the frequency at which information is printed.
    """

    # Initialize NodeC
    rospy.init_node("NodeC")
    
    # Get frequency from parameters (with a default value if not set)
    freq = rospy.get_param("freq", 1)  # Default value: 1 Hz
    
    # Set the publishing rate
    rate = rospy.Rate(freq)
    
    # Subscribe to the /robot_info topic to get the robot's information
    rospy.Subscriber("/robot_info", RobotPosition, callback)
    
    # Keep the node running, printing info at the specified frequency
    while not rospy.is_shutdown():
        rate.sleep()  # Control the frequency at which the callback is called

    # Keep the node running indefinitely
    rospy.spin()
    a = print(f'everything is working')

if __name__ == "__main__":
    main()
