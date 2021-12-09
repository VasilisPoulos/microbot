#!/usr/bin/env python

from numpy.core.defchararray import index
import rospy
import numpy as np
from geometry_msgs.msg import Twist 
from nav_msgs.msg import Odometry
from microrobot.srv import *
from tf.transformations import euler_from_quaternion

global _roll, _pitch, _yaw, _x, _y, _theta, _trajectory, _direction
_roll = _pitch = _yaw = _x = _y = _theta = 0.0
_trajectory = []
_direction = []


def odom_callback(msg):   
    global _roll, _pitch, _yaw, _x, _y, _theta
    _x = msg.pose.pose.position.x
    _y = msg.pose.pose.position.y
    rot_q = msg.pose.pose.orientation
    (_roll, _pitch, _theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])


# def trajectory_callback(data):
#     global _trajectory, _direction
#     converted_trajectory = (np.array(data.trajectory)).reshape(-1, 3).tolist()
#     _trajectory = converted_trajectory
#     _direction = data.direction
#     rospy.loginfo('Got: %s trajectory: %d direction: %d', data.message, len(converted_trajectory), \
#         len(data.direction))


def clothoid_generator_client(trajectory_points):
    rospy.wait_for_service('clothoid_generator')
    try:
        trajectory_points_1d = np.array(trajectory_points).flatten()
        generate_clothoids = rospy.ServiceProxy('clothoid_generator', Trajectory)
        res = generate_clothoids(trajectory_points_1d)
        converted_trajectory = np.array(res.trajectory).reshape(-1, 3).tolist()
        return converted_trajectory, res.direction
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


def start_controller():
    global _trajectory, _direction

    rospy.init_node('controller', anonymous=False)
    rate = rospy.Rate(100)  # 10hz
    
    trajectory_points = [
            np.array([1., 1., 1.]), 
            np.array([-0.5, -0.6, 1.]),
            np.array([-7., -15., 1.]),
            np.array([25., -2., 1.]),
            np.array([12., -1., 1.]),
            np.array([17., 4., 1.]),
            np.array([5., -1., 1.])]

    _trajectory, _direction = clothoid_generator_client(trajectory_points)
    odom_sub = rospy.Subscriber('/odom', Odometry, odom_callback)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    twist_msg = Twist()
    
    i = 0
    x_des = 1#_trajectory[0][0]
    y_des = 1#_trajectory[0][1]
    theta_des = np.radians(_direction[0])
    Kr = 0.07
    Ka = 0.09
    Kb = 0.09
    width = 0.178

    while not rospy.is_shutdown():
        r = np.sqrt(((x_des - _x)**2) + ((y_des - _y)**2))
        alpha = (np.arctan2(y_des - _y, x_des - _x)) - _theta
        beta = _theta + alpha - theta_des
        v_des = Kr*r
        omega_des = ((Ka*alpha) + (Kb*beta)) / width

        twist_msg.linear.x = v_des #m/s
        twist_msg.linear.y = 0.0
        twist_msg.linear.z = 0.0

        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = omega_des #rad/s


        if (round(_x, 1) == x_des and \
            round(_y, 1) == y_des and \
            round(_theta, 1) == theta_des):
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0
            print('HEREHEREHEREHEREHEREHERE')
            # change x_des, y_des, theta_des\
            # plot when there speed acceleration etc.
            if i < len(trajectory_points):
                x_des = _trajectory[i][0]
                y_des = _trajectory[i][1]
                theta_des = np.radians(_direction[i])

        rospy.loginfo('\nx %f y %f theta %f\nlinear x %f, y %f, z %f \nangular x %f,\
            y %f, z %f\ntarget%d: %f %f th: %f'\
            , _x, _y, _theta,\
            twist_msg.linear.x, twist_msg.linear.y, twist_msg.linear.z,\
            twist_msg.angular.x, twist_msg.angular.y, twist_msg.angular.z,
            i, x_des, y_des, theta_des)

        pub.publish(twist_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        start_controller()
    except rospy.ROSInterruptException:
        pass