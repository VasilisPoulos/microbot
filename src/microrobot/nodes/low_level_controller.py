#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Twist 
from gazebo_msgs.msg import ModelStates

# def callback(data):
#     rospy.loginfo(rospy.get_caller_id() + " heard linear %s angular %s", 
#         data.linear.x, data.angular.z)

def model_speed(msg):

    v_x = msg.twist[1].linear.x
    v_y = msg.twist[1].linear.y
    v = math.sqrt(v_x ** 2 + v_y ** 2)

    rospy.loginfo(rospy.get_caller_id() + "speed: %s", round(v, 8))

if __name__ == '__main__':
    try:
        rospy.init_node("low_level_controller")
        rate = rospy.Rate(1)
        rospy.loginfo('started low_level_controller node')
        
        # cmd_vel = rospy.Subscriber('/microrobot/cmd_vel', Twist, callback)
        #start_time = rospy.get_time()
        #rospy.loginfo("listening %s".format(start_time))

        while not rospy.is_shutdown():
            sub = rospy.Subscriber("/gazebo/model_states", ModelStates, model_speed)
            rate.sleep()

    except rospy.ROSInterruptException:
        pass