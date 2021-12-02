#!/usr/bin/env python

import rospy
from microrobot.msg import Trajectory
import numpy as np

import sys
sys.path.insert(0, '/home/vasilisp/catkin_ws/src/microrobot/src/scripts')

from clothoids import clothoid_trajectory

if __name__ == '__main__':
    try:
        pub = rospy.Publisher('trajectory', Trajectory, queue_size=10)
        rospy.init_node('trajectory_publisher', anonymous=True)
        rate = rospy.Rate(10)  # 10hz

        trajectory_points = [np.array([-20., -1., 1.]), 
                    np.array([-5., -6., 1.]),
                    np.array([-7., -15., 1.]),
                    np.array([25., -2., 1.]),
                    np.array([12., -1., 1.]),
                    np.array([17., 4., 1.]),
                    np.array([5., -1., 1.])]

        trajectory, direction = clothoid_trajectory(trajectory_points)
        trajectory = np.array(trajectory)
        print(len(trajectory), len(direction))
        while not rospy.is_shutdown():
            connections = pub.get_num_connections()

            msg = Trajectory()
            msg.message = 'cloth 0'
            msg.trajectory = list(trajectory.flatten())
            msg.direction = direction

            pub.publish(msg)

            rospy.loginfo('Connections: %d trajectory: %d direction: %d', connections, \
                len(trajectory), len(direction)) 
            
            if connections > 0:
                
               rospy.loginfo('Published')
               break
            rate.sleep()
    except rospy.ROSInterruptException:
        pass