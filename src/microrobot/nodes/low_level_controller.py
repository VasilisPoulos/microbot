#!/usr/bin/env python

import rospy
import math
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from tf.transformations import euler_from_quaternion, quaternion_from_euler

left_motor_topic = "/microbot/left_joint_velocity_controller/command"
right_motor_topic = "/microbot/right_joint_velocity_controller/command"
odometry_topic = "/visual_odometry"
omega_motor_L = 0
omega_motor_R = 0
x_current = 0
y_current = 0
theta_current = 0

x_desired = 0.047
y_desired = -0.089
theta_desired_deg = 90 # in deg
theta_desired = math.radians(theta_desired_deg)

motor_bias = 1400 # min speed for robot to move
Kp_position = 1000
Kp_orientation = 400

def reset_speed():
    left_motor_publisher.publish(0.0)
    right_motor_publisher.publish(0.0)
    rospy.loginfo('resetting speed...')

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

def robot_reached_target(msg, x_desired, y_desired):
    limit = 0.0005 # positive number for symmetric limits
    x_current = msg.pose.pose.position.x
    y_current = msg.pose.pose.position.y
    if x_desired + limit > x_current > x_desired - limit and \
       y_desired + limit > y_current > y_desired - limit:
       return True
    return False

def robot_reached_theta(theta_des, theta_current, acceptance_range):
    if theta_des + acceptance_range > theta_current > theta_des - acceptance_range:
       return True
    return False

def limiter(limit):
    global omega_motor_R, omega_motor_L

    if omega_motor_L > limit:
        omega_motor_L = limit

    if omega_motor_R > limit:
        omega_motor_R = limit

def on_site_rotation_to(theta_desired):
    global omega_motor_R, omega_motor_L

    theta_error = theta_desired - theta_current
    omega_diff = Kp_orientation * abs((theta_desired - theta_current)) + motor_bias
    
    if theta_current < theta_desired:
        omega_motor_L =   omega_diff
        omega_motor_R = - omega_diff
    else:
        omega_motor_L = - omega_diff
        omega_motor_R =   omega_diff
    
    limiter(1800)

    rospy.loginfo("rpmL: %f, rpmR: %f, theta_current: %f, omega_diff: %f theta_error: %f", \
        omega_motor_L, omega_motor_R, math.degrees(theta_current), omega_diff, theta_error)

    if robot_reached_theta(theta_desired_deg, math.degrees(theta_current), 0.01):
        reset_speed()
        rospy.loginfo('Target reached')
        return

def controller(msg):
    
    update_pose_and_orientation(msg)

    on_site_rotation_to(theta_desired)

    # x_error = x_desired - x_current
    # y_error = y_desired - y_current
    # euclidean_distance_error = math.sqrt(((x_error)**2) + ((y_error)**2))
    
    #omega_base = Kp_position * euclidean_distance_error + motor_bias
    
   

if __name__ == '__main__':
    try:
        rospy.init_node("low_level_controller")
        rospy.loginfo('Starting.')
        rate = rospy.Rate(10)
        rospy.on_shutdown(reset_speed)

        left_motor_publisher = rospy.Publisher(left_motor_topic, Float64, queue_size=1)
        right_motor_publisher = rospy.Publisher(right_motor_topic, Float64, queue_size=1)
        odometry_subscriber = rospy.Subscriber(odometry_topic, Odometry, controller)
        
        while not rospy.is_shutdown():
            left_motor_publisher.publish(omega_motor_L)
            right_motor_publisher.publish(-omega_motor_R)
            rate.sleep()

    except rospy.ROSInterruptException:
        pass