#!/usr/bin/env python3
# Import necessary libraries
import rospy
import math
from geometry_msgs.msg import Twist
from turtlesim.srv import SetPen
from turtlesim.msg import Pose

def move_turtle(turtle_name, x, y, count):
    # Set the pen to down
    rospy.wait_for_service('/' + turtle_name + '/set_pen')
    set_pen = rospy.ServiceProxy('/' + turtle_name + '/set_pen', SetPen)
    set_pen(255, 255, 255, 5, 0)
    
    # Create publisher object to send velocity commands
    pub = rospy.Publisher('/' + turtle_name + '/cmd_vel', Twist, queue_size=10)
    
    # Set rate of publishing commands
    rate = rospy.Rate(10)
    
    # Get current position of turtle
    current_position = rospy.wait_for_message("/" + turtle_name + "/pose", Pose)
    
    # Calculate the distance to travel
    distance = ((x - current_position.x)**2 + (y - current_position.y)**2)**0.5
    
    # Create Twist message object
    velocity_command = Twist()
    
    # Continuously publish velocity commands until goal is reached
    while distance > 0.1:
        # Calculate the angle between turtle's heading and desired direction
        desired_angle = math.atan2(y - current_position.y, x - current_position.x)
        angle_diff = desired_angle - current_position.theta
        angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))
        
        # Set angular velocity in z-axis
        velocity_command.angular.z = 2 * angle_diff
        
        # Limit the maximum angular velocity
        if velocity_command.angular.z > 0.5:
            velocity_command.angular.z = 0.5
        elif velocity_command.angular.z < -0.5:
            velocity_command.angular.z = -0.5
        
        # Set linear velocity in x-axis
        velocity_command.linear.x = 0.5
        
        pub.publish(velocity_command)
        rate.sleep()
        current_position = rospy.wait_for_message("/" + turtle_name + "/pose", Pose)
        distance = ((x - current_position.x)**2 + (y - current_position.y)**2)**0.5
    
    # Stop the turtle
    velocity_command.linear.x = 0
    velocity_command.angular.z = 0
    pub.publish(velocity_command)

if __name__ == '__main__':
    count = 0
    rospy.init_node('move_turtle', anonymous=True)
    try:
        while True:
            turtle = input("Enter the turtle (a, b, or c): ")
            if turtle == 'a':
                turtle_name = 'turtle1'
            elif turtle == 'b':
                turtle_name = 'turtle2'
            elif turtle == 'c':
                turtle_name = 'turtle3'
            else:
                print("Invalid turtle name. Enter a, b, or c.")
                continue
            x = float(input("Enter x coordinate: "))
            y = float(input("Enter y coordinate: "))
            move_turtle(turtle_name, x, y, count)
            count += 1
    except rospy.ROSInterruptException:
        pass

# if __name__ == '__main__':
#     try:
#         while True:
#             turtle = input("Enter the turtle (a, b, or c): ")
#             x = float(input("Enter x coordinate: "))
# # Continue the code
#             y = float(input("Enter y coordinate: "))
#             if turtle == 'a':
#                 move_turtle_to_goal('turtle1', x, y)
#             elif turtle == 'b':
#                 move_turtle_to_goal('turtle2', x, y)
#             elif turtle == 'c':
#                 move_turtle_to_goal('turtle3', x, y)
#             else:
#                 print("Invalid turtle name. Try again.")
#     except rospy.ROSInterruptException:
#         pass
