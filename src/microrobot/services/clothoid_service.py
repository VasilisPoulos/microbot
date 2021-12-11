#!/usr/bin/env python

import rospy
import numpy as np
import re
from microrobot.srv  import Trajectory, TrajectoryResponse
from microrobot.scripts import clothoids
# import sys
# sys.path.insert(0, '/home/vasilisp/catkin_ws/src/microrobot/src/scripts')
# from clothoids import clothoid_trajectory

def generate_clothoid_trajectory(trajectory_request):
    print('Sent..')
    pattern = re.compile(r'-?\d+\.\d+')
    trajectory_points = []
    for match in pattern.finditer(str(trajectory_request)):
        trajectory_points.append(float(match.group()))

    converted_trajectory_points = np.array(trajectory_points).reshape(-1, 3).tolist()
    trajectory, direction = clothoids.clothoid_trajectory(converted_trajectory_points)
    converted_trajectory = np.array(trajectory).flatten().tolist()
    return TrajectoryResponse(converted_trajectory, direction)

def clothoid_server():
    rospy.init_node('clothoid_generator_server', anonymous=True)
    rospy.Service('clothoid_generator', Trajectory, generate_clothoid_trajectory)
    print('Ready to generate clothoid trajectories.')
    rospy.spin()

if __name__ == '__main__':
    try:
        clothoid_server()
    except rospy.ROSInterruptException:
        pass