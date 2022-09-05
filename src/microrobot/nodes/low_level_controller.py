#!/usr/bin/env python

import rospy
import math
#from geometry_msgs.msg import Twist 
#from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
omega_motor = 0
# def callback(data):
#     rospy.loginfo(rospy.get_caller_id() + " heard linear %s angular %s", 
#         data.linear.x, data.angular.z)

def model_speed(msg):
    global omega_motor
    motor_bias = 1000
    K_c = 5000
    v_desired = 0.005
    v_current = msg.twist.twist.linear.x
    
    omega_motor = K_c * (v_desired - v_current) + motor_bias

    rospy.loginfo("omega: %f, linear_speed: %f", omega_motor, v_current)

if __name__ == '__main__':
    try:
        rospy.init_node("low_level_controller")
        rate = rospy.Rate(10)
        rospy.loginfo('started low_level_controller node')
        
        left_motor_topic = "/microbot/left_joint_velocity_controller/command"
        right_motor_topic = "/microbot/right_joint_velocity_controller/command"

        left_motor_publisher = rospy.Publisher(left_motor_topic, Float64, queue_size=1)
        right_motor_publisher = rospy.Publisher(right_motor_topic, Float64, queue_size=1)
        odometry_subscriber = rospy.Subscriber("/odom", Odometry, model_speed)
        
        while not rospy.is_shutdown():
            left_motor_publisher.publish(omega_motor)
            right_motor_publisher.publish(-omega_motor)
            rate.sleep()

    except rospy.ROSInterruptException:
        pass