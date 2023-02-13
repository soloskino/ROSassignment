#!/usr/bin/env python3
import rospy
import random
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

def turtle_pose_callback(pose, turtle_id):
    global x, y, theta
    x[turtle_id] = pose.x
    y[turtle_id] = pose.y
    theta[turtle_id] = pose.theta

def random_motion_with_wall_avoidance(turtle_id):
    global x, y, theta
    pub = rospy.Publisher('/turtle' + str(turtle_id + 1) + '/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/turtle' + str(turtle_id + 1) + '/pose', Pose, lambda pose: turtle_pose_callback(pose, turtle_id))

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        # check if this turtle is colliding with another turtle
        collision = False
        for i in range(3):
            # don't check for collision with itself
            if i != turtle_id:
                distance = ((x[turtle_id] - x[i])**2 + (y[turtle_id] - y[i])**2)**0.5
                if distance < 0.5: # adjust the distance threshold as needed
                    # if collision detected, turn the turtle away from the other turtle
                    collision = True
                    break

        if collision:
            twist = Twist()
            twist.linear.x = 0.5
            twist.angular.z = random.uniform(-2, 2)
            pub.publish(twist)
            continue
          
        # check if the turtle is close to the wall
        if x[turtle_id] < 1.5:
            twist = Twist()
            twist.linear.x = 0.5
            twist.angular.z = random.uniform(1, 2)
            pub.publish(twist)
        elif x[turtle_id] > 9.5:
            twist = Twist()
            twist.linear.x = 0.5
            twist.angular.z = random.uniform(-2, -1)
            pub.publish(twist)
        elif y[turtle_id] < 1.5:
            twist = Twist()
            twist.linear.x = 0.5
            twist.angular.z = random.uniform(0.5, 2)
            pub.publish(twist)
        elif y[turtle_id] > 9.5:
            twist = Twist()
            twist.linear.x = 0.5
            twist.angular.z = random.uniform(-2, -0.5)
            pub.publish(twist)
        else:
            twist = Twist()
            twist.linear.x = random.uniform(0, 2)
            twist.angular.z = random.uniform(-2, 2)
            pub.publish(twist)

        rate.sleep()

if __name__ == '__main__':
    # Initialize the node
    rospy.init_node('random_motion_with_wall_avoidance')

    # Declare global variables
    x = [0, 0, 0]
    y = [0, 0, 0]
    theta = [0, 0, 0]

    # Start the random motion for each turtle
    for i in range(3):
        rospy.loginfo("Starting random motion for turtle" + str(i + 1))
        rospy.Timer(rospy.Duration(i + 1), lambda x, i=i: random_motion_with_wall_avoidance(i))


    # Spin to keep the node running
    rospy.spin()

# Gracefully shutdown the node when interrupted
rospy.on_shutdown(lambda: rospy.loginfo("Node has been shutdown"))


