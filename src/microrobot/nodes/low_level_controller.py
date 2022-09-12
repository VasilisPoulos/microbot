#!/usr/bin/env python

import rospy
import math
#from geometry_msgs.msg import Twist 
#from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
omega_motor_L = 0
omega_motor_R = 0
# def callback(data):
#     rospy.loginfo(rospy.get_caller_id() + " heard linear %s angular %s", 
#         data.linear.x, data.angular.z)

def reset_speed():
    left_motor_publisher.publish(0.0)
    right_motor_publisher.publish(0.0)
    rospy.loginfo('resetting speed...')

def model_speed(msg):
    global omega_motor_L, omega_motor_R
    x_desired = 0.3
    y_desired = 0.3
    theta_desired = 0.30 
    v_desired = 0.0005

    motor_bias = 2500
    K_p1 = 11000
    K_p2 = 1100

    x_current = msg.pose.pose.position.x
    y_current = msg.pose.pose.position.y
    theta_current = msg.pose.pose.orientation.z
    v_current = msg.twist.twist.linear.x

    x_error = x_desired - x_current
    y_error = y_desired - y_current
    theta_error = theta_desired - theta_current
    
    #omega_motor = K_p1 * (v_desired - v_current)
    omega_diff =  abs(K_p2 * (theta_desired - theta_current))
    omega_motor = 2800 #K_p2 * (x_error)

    omega_motor_R = omega_motor_L = omega_motor
    if theta_current > theta_desired:
        omega_motor_L = omega_motor + omega_diff
    else:
        omega_motor_R = omega_motor + omega_diff
    

   
    rospy.loginfo("rpmL: %f, rpmR: %f, omega_diff: %f x: %f y: %f theta_error: %f", \
        omega_motor_L, omega_motor_R, omega_diff, x_current, y_current, theta_error)

if __name__ == '__main__':
    try:
        rospy.init_node("low_level_controller")
        rate = rospy.Rate(10)
        rospy.loginfo('started low_level_controller node')
        rospy.on_shutdown(reset_speed)

        left_motor_topic = "/microbot/left_joint_velocity_controller/command"
        right_motor_topic = "/microbot/right_joint_velocity_controller/command"

        left_motor_publisher = rospy.Publisher(left_motor_topic, Float64, queue_size=1)
        right_motor_publisher = rospy.Publisher(right_motor_topic, Float64, queue_size=1)
        odometry_subscriber = rospy.Subscriber("/odom", Odometry, model_speed)
        
        while not rospy.is_shutdown():
            left_motor_publisher.publish(omega_motor_L)
            right_motor_publisher.publish(-omega_motor_R)
            rate.sleep()

    except rospy.ROSInterruptException:
        pass