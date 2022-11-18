#!/usr/bin/env python

import rospy
import math
import numpy as np
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from tf.transformations import euler_from_quaternion, quaternion_from_euler

left_motor_topic = "/microbot/left_joint_velocity_controller/command"
right_motor_topic = "/microbot/right_joint_velocity_controller/command"
odometry_topic = "/visual_odometry"
motor_L = 0
motor_R = 0
x_current = 0
y_current = 0
theta_current = 0

x_desired = 0.03
y_desired = 0.03
theta_desired_deg = 0 # in deg
theta_desired = math.radians(theta_desired_deg)

motor_bias = 1400 # min speed for robot to move

def stop_motors():
    left_motor_publisher.publish(0.0)
    right_motor_publisher.publish(0.0)
    rospy.loginfo('Stopping motors.')

def update_pose_and_orientation(msg):
    global x_current, y_current, theta_current

    robot_position = msg.pose.pose.position
    robot_orientation = msg.pose.pose.orientation

    x_current = robot_position.x
    y_current = robot_position.y

    orientation_list = [robot_orientation.x, robot_orientation.y, \
        robot_orientation.z, robot_orientation.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)

    theta_current = yaw

def robot_reached_theta(theta_des, theta_current, acceptance_range):
    if theta_des + acceptance_range > theta_current > theta_des - acceptance_range:
       return True
    return False

def robot_reached_target(x_desired, y_desired, theta_des, acceptance_range=0.009):

    if x_desired + acceptance_range > x_current > x_desired - acceptance_range and \
       y_desired + acceptance_range > y_current > y_desired - acceptance_range and \
       robot_reached_theta(theta_des, theta_current, 0.09):
       return True
    return False

# def smooth_set_range(current_rpm, rpm, step, motor_name):
#     global motor_R, motor_L

#     rpm_list = np.linspace(current_rpm, rpm, step)
#     for rpm in rpm_list:
#         rospy.loginfo("motor: %f", rpm)
#         # if motor_name == "L":
#         motor_L = rpm
#         # else:
#         #     motor_R = rpm
#         rospy.sleep(0.001)
        
def limiter(limit):
    global motor_R, motor_L

    if motor_L > limit:
        motor_L = limit
    elif motor_L < - limit:
        motor_L = - limit

    if motor_R > limit:
        motor_R = limit
    elif motor_R < - limit:
        motor_R = - limit

def on_site_rotation_to(theta_desired, accuracy=0.09):
    global motor_R, motor_L

    # P controller
    Kp_orientation = 400
    theta_error = theta_desired - theta_current
    omega_diff = Kp_orientation * abs((theta_error)) + motor_bias
    
    if theta_current < theta_desired:
        motor_L =   omega_diff
        motor_R = - omega_diff
    else:
        motor_L = - omega_diff
        motor_R =   omega_diff
    
    limiter(1800)

    rospy.loginfo("rpmL: %f, rpmR: %f, theta_current: %f, omega_diff: %f theta_error: %f", \
        motor_L, motor_R, math.degrees(theta_current), omega_diff, theta_error)

    if robot_reached_theta(theta_desired_deg, math.degrees(theta_current), accuracy):
        # stop_motors()
        rospy.loginfo('Target reached')
        # TODO: Following line should be removed.
        rospy.signal_shutdown('Target reached')

def go_to(x_desired, y_desired, theta_desired, accuracy=0.09):
    global motor_R, motor_L

    # P controller
    Kp_position = 400
    Kp_orientation = 1000
    Kp_a = 300
    Kp_b = 800
    tmp_motor_L = 0
    tmp_motor_R = 0

    x_error = x_desired - x_current
    y_error = y_desired - y_current
    theta_error = theta_desired - theta_current
    euclidean_distance_error = math.sqrt(((x_error)**2) + ((y_error)**2)) 
    alpha = math.atan(y_error/x_error) - theta_current
    beta = theta_current + alpha - theta_desired


    rpm = Kp_position * euclidean_distance_error + 2000
    rpm_diff = Kp_a * alpha + Kp_b * beta
    
    if rpm_diff > 0:
        tmp_motor_R = - (rpm + rpm_diff)
        tmp_motor_L = - rpm_diff
    else:
        tmp_motor_R = - rpm_diff
        tmp_motor_L = - (rpm - rpm_diff)

    if 10 > math.degrees(alpha) > -10 and euclidean_distance_error > 0.005:
        if 10 > math.degrees(alpha): 
            tmp_motor_R = - 2000
            tmp_motor_L = - 2100
        elif math.degrees(alpha) > 10:
            tmp_motor_R = - 2000
            tmp_motor_L = - 2100
        else:
            tmp_motor_R = - 2000
            tmp_motor_L = - 2000

    if tmp_motor_R > 0 and motor_R < 0 or \
       tmp_motor_R < 0 and motor_R > 0 or \
       tmp_motor_L > 0 and motor_L < 0 or \
       tmp_motor_L < 0 and motor_L > 0:
        # if change in direction of motors
        stop_motors()
        # to prevent crashes.

    motor_R = tmp_motor_R
    motor_L = tmp_motor_L

    limiter(2200)
    rospy.loginfo("rpmL: %f, rpmR: %f, x_current: %f, y_current: %f theta_current: %f\nrpm_diff: %f euclidean_error: %f a: %f b: %f", \
        motor_L, motor_R, x_current, y_current, 
        math.degrees(theta_current), rpm_diff, euclidean_distance_error, \
        math.degrees(alpha), math.degrees(beta))

    if robot_reached_target(x_desired, y_desired, theta_desired_deg):
        # stop_motors()
        rospy.loginfo('Target reached')
        # TODO: Following line should be removed.
        rospy.signal_shutdown('Target reached')

def controller(msg):
    
    update_pose_and_orientation(msg)

    #on_site_rotation_to(theta_desired)
    go_to(x_desired, y_desired, theta_desired)

if __name__ == '__main__':
    try:
        rospy.init_node("low_level_controller")
        rospy.loginfo('Starting.')
        rate = rospy.Rate(10)
        rospy.on_shutdown(stop_motors)

        left_motor_publisher = rospy.Publisher(left_motor_topic, Float64, queue_size=1)
        right_motor_publisher = rospy.Publisher(right_motor_topic, Float64, queue_size=1)
        odometry_subscriber = rospy.Subscriber(odometry_topic, Odometry, controller)
        
        while not rospy.is_shutdown():
            left_motor_publisher.publish(-motor_L)
            right_motor_publisher.publish(motor_R)
            rate.sleep()

    except rospy.ROSInterruptException:
        pass