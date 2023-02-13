#!/usr/bin/env python3
import rospy
import sys
import termios
import tty
from geometry_msgs.msg import Twist
from turtlesim.srv import TeleportAbsolute
from turtlesim.msg import Pose




def get_key():
    # Get the file descriptor of the terminal
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    # Change the terminal settings
    tty.setraw(sys.stdin.fileno())
    ch = sys.stdin.read(1)

    # Reset the terminal settings
    termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    return ch

def get_position(turtle_num):
    if turtle_num == 1:
        pose = rospy.wait_for_message("/turtle1/pose", Pose)
    elif turtle_num == 2:
        pose = rospy.wait_for_message("/turtle2/pose", Pose)
    elif turtle_num == 3:
        pose = rospy.wait_for_message("/turtle3/pose", Pose)
    return pose.x, pose.y

def keyboard_input(turtle_num):
    # Print the instructions for the user
    print("\nControl the turtle with the following keys:")
    print("t: Move up")
    print("v: Move down")
    print("f: Move left")
    print("g: Move right")
    print("y: Move clockwise")
    print("r: Move anti-clockwise")
    print("i: Control turtle1")
    print("o: Control turtle2")
    print("p: Control turtle3")
    print("u: Control all turtles")
    print("c: Stop\n")

    # Get the keyboard input
    key = get_key()
    velocity_msg = Twist()


    # Set the velocity based on the key
    if key == 't':
        velocity_msg.linear.x = 1.0
        print("Moving up")
    elif key == 'v':
        velocity_msg.linear.x = -1.0
        print("Moving down")
    elif key == 'f':
        velocity_msg.angular.z = 1.0
        print("Moving left")
    elif key == 'g':
        velocity_msg.angular.z = -1.0
        print("Moving right")
    elif key == 'y':
        velocity_msg.angular.z = 1.0
        velocity_msg.linear.x = 1.0
        print("Moving clockwise")
    elif key == 'r':
        velocity_msg.angular.z = -1.0
        velocity_msg.linear.x = -1.0
        print("Moving anti-clockwise")
    elif key == 'c':
        velocity_msg.angular.z = 0
        velocity_msg.linear.x = 0
        print("Stopped")
    elif key == 'i':
        print("Controlling turtle1")
        turtle_num = 1
    elif key == 'o':
        print("Controlling turtle2")
        turtle_num = 2
    elif key == 'p':
        print("Controlling turtle3")
        turtle_num = 3
    elif key == 'u':
        print("Controlling all turtles")
        turtle_num = 4
    else:
        print("Invalid key")
        sys.exit()

    return turtle_num, velocity_msg

def move_turtle(turtle_num, velocity_msg):
    x, y = get_position(turtle_num)
    if x > 9 or y > 9 :
        velocity_msg.linear.x = -velocity_msg.linear.x
        velocity_msg.angular.z = -velocity_msg.angular.z

    if turtle_num == 1:
        velocity_publisher1.publish(velocity_msg)
    elif turtle_num == 2:
        velocity_publisher2.publish(velocity_msg)
    elif turtle_num == 3:
        velocity_publisher3.publish(velocity_msg)
    elif turtle_num == 4:
        velocity_publisher1.publish(velocity_msg)
        velocity_publisher2.publish(velocity_msg)
        velocity_publisher3.publish(velocity_msg)
    else:
        print("Invalid turtle number")
        sys.exit()



if __name__ == '__main__':
    turtle_num = 1
    # Initialize the node
    rospy.init_node('gpt_teleop', anonymous=True)
    velocity_publisher1 = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    velocity_publisher2 = rospy.Publisher('/turtle2/cmd_vel', Twist, queue_size=10)
    velocity_publisher3 = rospy.Publisher('/turtle3/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        turtle_num, velocity_msg = keyboard_input(turtle_num)
        move_turtle(turtle_num, velocity_msg)
        rate.sleep()

