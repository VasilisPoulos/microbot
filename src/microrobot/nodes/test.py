#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64

def reset_speed():
    left_motor_publisher.publish(0.0)
    right_motor_publisher.publish(0.0)
    rospy.loginfo('resetting speed...')

if __name__ == '__main__':
    try:
        rospy.init_node("testing_node")
        rate = rospy.Rate(10)
        rospy.loginfo('started testing node')
        
        left_motor_publisher = rospy.Publisher( \
            '/microbot/left_joint_velocity_controller/command', \
            Float64, queue_size=1)
        right_motor_publisher = rospy.Publisher( \
            '/microbot/right_joint_velocity_controller/command', \
            Float64, queue_size=1)

        rospy.loginfo('publishing')

        rospy.on_shutdown(reset_speed)

        while not rospy.is_shutdown():
            left_motor_publisher.publish(5000.0)
            right_motor_publisher.publish(0)
            rate.sleep()

    except rospy.ROSInterruptException:
        pass