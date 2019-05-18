#!/usr/bin/env python

import rospy

from geometry_msgs.msg import Twist

PI = 3.1415926535897


class movement :

    def __init__(self):
        rospy.init_node('move_robot_node', anonymous=False)
        self.pub_move = rospy.Publisher("/cmd_vel",Twist, queue_size=1)
        self.rate = rospy.Rate(1)


    def move_forward(self):
        print "move forward"
        move = Twist()
        move.linear.x = 1.0 
        move.angular.z = 0.0
        for i in range(10):
            move.linear.x += 0.1
            self.pub_move.publish(move)
            self.rate.sleep()
            print move
        print "stop"
        move.linear.x = 0
        while not rospy.is_shutdown():
            self.pub_move.publish(move)
            self.rate.sleep()
            print move


    def move_backward(self):
        move =Twist()
        move.linear.x=-1
        move.angular.z=0.0
        self.pub_move.publish(move)
    def stop(self):
        move =Twist()
        move.linear.x=0
        move.angular.z=0.0
        self.pub_move.publish(move)

    def rotate_right(self):
        move =Twist()
        # Receiveing the user's input
        print("Let's rotate your robot")
        speed = 100
        angle = 90
        clockwise = True #True or false

        #Converting from angles to radians
        angular_speed = speed*2*PI/360
        relative_angle = angle*2*PI/360

            #We wont use linear components
        move.linear.x=0
        move.linear.y=0
        move.linear.z=0
        move.angular.x = 0
        move.angular.y = 0

            # Checking if our movement is CW or CCW
        if clockwise:
            move.angular.z = -abs(angular_speed)
            print move.angular.z
        else:
            move.angular.z = abs(angular_speed)
        # Setting the current time for distance calculus
        t0 = rospy.Time.now().to_sec()
        current_angle = 0

        while(current_angle < relative_angle):
            self.pub_move.publish(move)
            t1 = rospy.Time.now().to_sec()
            current_angle = angular_speed*(t1-t0)
        #Forcing our robot to stop
        move.angular.z = 0
        self.pub_move.publish(move)
        rospy.spin()

while not rospy.is_shutdown() :

    mov = movement()
    mov.move_forward()
    rospy.spin()