#!/usr/bin/env python
import rospy
import numpy as np
from microrobot.msg import Trajectory
from geometry_msgs.msg import Twist 
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler


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


def trajectory_callback(data):
    global _trajectory, _direction
    converted_trajectory = (np.array(data.trajectory)).reshape(-1, 3).tolist()
    _trajectory = converted_trajectory
    _direction = data.direction
    rospy.loginfo('Got: %s trajectory: %d direction: %d', data.message, len(converted_trajectory), \
        len(data.direction))


def start_controller():
    rospy.init_node('controller', anonymous=False)
    rate = rospy.Rate(100)  # 10hz
    # cloth_sub = rospy.Subscriber('/odom', Trajectory, trajectory_callback)
    odom_sub = rospy.Subscriber('/odom', Odometry, odom_callback)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    twist_msg = Twist()
    
    x_des = 0.4 #_trajectory[0][0]
    y_des = 0.4 #_trajectory[0][0]
    theta_des = 0.0 #* np.pi
    Kr = 0.07
    Ka = 0.009
    Kb = 0.009
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

        rospy.loginfo('\nx %f y %f theta %f\nlinear x %f, y %f, z %f \nangular x %f, y %f, z %f\n'\
            , _x, _y, _theta,\
            twist_msg.linear.x, twist_msg.linear.y, twist_msg.linear.z,\
            twist_msg.angular.x, twist_msg.angular.y, twist_msg.angular.z)

        pub.publish(twist_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        start_controller()
    except rospy.ROSInterruptException:
        pass