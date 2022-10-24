#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64

def reset_speed():
    left_motor_publisher.publish(0.0)
    right_motor_publisher.publish(0.0)
    rospy.loginfo('resetting speed...')

l_rpm = 1600.0
r_rpm = 1600.0
test_mode = False

if __name__ == '__main__':
    try:
        rospy.init_node("testing_node")
        rate = rospy.Rate(10)
        
        left_motor_publisher = rospy.Publisher( \
            '/microbot/left_joint_velocity_controller/command', \
            Float64, queue_size=1)
        right_motor_publisher = rospy.Publisher( \
            '/microbot/right_joint_velocity_controller/command', \
            Float64, queue_size=1)        

        left_set_point_publisher = rospy.Publisher( \
            '/left_set_point', \
            Float64, queue_size=1)    
        
        rospy.on_shutdown(reset_speed)

        while not rospy.is_shutdown():

            if test_mode:
                l_rpm = float(input("new_rpm:"))
                r_rpm = l_rpm

            rospy.loginfo('l_rpm: %.1f r_rpm: %.1f', l_rpm, r_rpm)

            left_set_point_publisher.publish(l_rpm)
            left_motor_publisher.publish(l_rpm)
            right_motor_publisher.publish(r_rpm)
            rate.sleep()

    except rospy.ROSInterruptException:
        pass