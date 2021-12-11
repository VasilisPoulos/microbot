#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import Twist 
from nav_msgs.msg import Odometry
from microrobot.srv import *
from tf.transformations import euler_from_quaternion

global _roll, _pitch, _yaw, _x, _y, _theta, _trajectory, _direction, _linear_limit, _Kr, _Ka, \
    _Kb

_roll = _pitch = _yaw = _x = _y = _theta = 0.0
_linear_speed_limit = 0.15
_angular_speed_limit = 1.9
_trajectory = []
_direction = []
_Kr = 0.9 # drive towards b
_Ka = 0.6 # turn the robot direction to b
_Kb = 0.3 # turn the robot to its final direction.
_width = 0.178

def odom_callback(msg):   
    global _roll, _pitch, _yaw, _x, _y, _theta
    _x = msg.pose.pose.position.x
    _y = msg.pose.pose.position.y
    rot_q = msg.pose.pose.orientation
    (_roll, _pitch, _theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
    
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

def on_the_trajectory(trajectory_index):
   True if trajectory_index < len(_trajectory) else False

def log_info(twist_msg, trajectory_index, x_des, y_des, theta_des):
    rospy.loginfo('\nx %f y %f theta %f\nlinear x %f' + \
            '\nangular z %f\ntarget%d: %f %f th: %f'\
            , _x, _y, _theta, twist_msg.linear.x, twist_msg.angular.z, \
            trajectory_index, x_des, y_des, theta_des)

def start_controller():
    global _trajectory, _direction

    rospy.init_node('controller', anonymous=False)
    rate = rospy.Rate(10)  # 10hz
    
    trajectory_points = [[1., 1., 1.], [-0.5, -1., 1.]]

    _trajectory, _direction = clothoid_generator_client(trajectory_points)
    odom_sub = rospy.Subscriber('/odom', Odometry, odom_callback)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    twist_msg = Twist()
    
    next_trajectory_index = 0
    x_des = _trajectory[0][0]
    y_des = _trajectory[0][1]
    theta_des = 0 #np.radians(_direction[0])

    while not rospy.is_shutdown():
        r = np.sqrt(((x_des - _x)**2) + ((y_des - _y)**2))
        alpha = (np.arctan2(y_des - _y, x_des - _x)) - _theta
        beta = _theta + alpha - theta_des
        v_des = _Kr*r
        omega_des = ((_Ka*alpha) + (_Kb*beta)) / _width

        # Linear speed limiter.
        if v_des > _linear_speed_limit:
            twist_msg.linear.x = _linear_speed_limit
        else:
            twist_msg.linear.x = v_des # m/s

        # Angular speed limiter.
        if np.abs(omega_des) > np.abs(_angular_speed_limit):
            if omega_des > 0:
                twist_msg.angular.z = _angular_speed_limit
            else:
                twist_msg.angular.z = - _angular_speed_limit
        else:
            twist_msg.angular.z = omega_des # rad/s

        if (round(_x, 1) == round(x_des, 1) and \
            round(_y, 1) == round(y_des, 1) and \
            round(_theta, 1) == round(theta_des, 1)):
            
            next_trajectory_index += 1
            if on_the_trajectory(next_trajectory_index):
                x_des = _trajectory[next_trajectory_index][0]
                y_des = _trajectory[next_trajectory_index][1]
                theta_des = np.radians(_direction[next_trajectory_index])
            else:
                twist_msg.linear.x = 0.0
                twist_msg.angular.z = 0.0
                rospy.signal_shutdown('stop')
                exit()
        
        log_info(twist_msg, next_trajectory_index, x_des, y_des, theta_des)
        pub.publish(twist_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        start_controller()
    except rospy.ROSInterruptException:
        pass