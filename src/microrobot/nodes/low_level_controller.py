#!/usr/bin/env python

import rospy
import math
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from tf.transformations import euler_from_quaternion

# Ros topics
left_motor_topic = "/microbot/left_joint_velocity_controller/command"
right_motor_topic = "/microbot/right_joint_velocity_controller/command"
odometry_topic = "/visual_odometry"

# Initializing microbot parameters
# Speed per motor
motor_L = 0
motor_R = 0

# Initial position
x_init = -1
y_init = -1
theta_init = -1
init_is_set = False

# Current position
x_current = 0
y_current = 0
theta_current = 0

# Goal
# Linear equation variables for line that connects current position to desired
alpha = 1
beta = 0
gamma = 0
equation_is_set = False
x_desired = 0.02
y_desired = 0.02
theta_desired = 0

# Error
euclidean_error = 0
theta_error = 0

# Biases per type of movement
# On site
Kp_on_site = 300        # P gain for on site rotation
on_site_bias = 1250     # min motor speed for on site rotation
on_site_limit = 1300    # max motor speed to reduce bouncing
# Drive forward
Kp_drive = 2800         # P gain for drive forward
drive_bias = 1250       # min motor speed for drive forward
drive_limit = 1350      # max motor speed to reduce bouncing

# Utility functions
def set_init_pose():
    global x_init, y_init, init_is_set
    if not init_is_set:
        x_init = x_current
        y_init = y_current
        init_is_set = True
        rospy.loginfo('Initialized pose x: %f y: %f theta:%f ', x_current, \
            y_current, math.degrees(theta_current))

def reset_init_pose():
    global x_init, y_init, init_is_set
    x_init = -1
    y_init = -1
    init_is_set = False

def euclidean_distance(p, q):
    return math.sqrt(sum((px - qx) ** 2.0 for px, qx in zip(p, q)))

def log_pose():
    rospy.loginfo("rpmL: %.0f, rpmR: %.0f, x_current: %f, y_current: %f theta_current: %.2f", \
        motor_L, motor_R, x_current, y_current, 
        math.degrees(theta_current))

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

def update_errors():
    global euclidean_error, theta_error
    euclidean_error =  euclidean_distance([x_current, y_current], \
        [x_desired, y_desired])
    theta_error = theta_desired - theta_current

def robot_reached_theta(theta_des, theta_current, acceptance_range):
    if theta_des + acceptance_range > theta_current > theta_des - acceptance_range:
       return True
    return False
        
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

def set_line_to_desired_position():
    global theta_desired, alpha, beta, gamma, equation_is_set

    if not equation_is_set:
        d_x = x_desired - x_current
        d_y = y_desired - y_current
        theta_desired = math.atan2(d_y, d_x)
        direction = d_y / d_x
        alpha = 1
        beta = - direction
        gamma = - y_current - direction * x_current
        equation_is_set = True

# Movement functions
def stop_motors():
    left_motor_publisher.publish(0.0)
    right_motor_publisher.publish(0.0)
    rospy.loginfo('Stopping motors.')

def on_site_rotation_to(theta_desired, accuracy=0.02):
    global motor_R, motor_L

    # P controller
    rpm = Kp_on_site * abs((theta_error)) + on_site_bias
    if theta_current > theta_desired:
        motor_L =   - rpm
        motor_R =   - rpm
    else:
        motor_L =   rpm
        motor_R =   rpm

    limiter(on_site_limit)

    if robot_reached_theta(theta_desired, theta_current, accuracy):
        rospy.loginfo('Target theta reached')
        return False
    else:
        rospy.loginfo("theta_desired: %f, theta_error: %f", \
            math.degrees(theta_desired), math.degrees(theta_error))
    return True

def drive_forward():
    global motor_L, motor_R
    
    # P controller
    rpm = Kp_drive*euclidean_error + drive_bias
    motor_L = - rpm
    motor_R = rpm
    limiter(drive_limit)

def go_to(x_desired, y_desired):
    global motor_R, motor_L, init_alpha_des, alpha, beta, gamma, theta_desired

    l = alpha*x_current + beta*y_current + gamma \
        / math.sqrt(alpha**2 + beta**2)

    if not (on_site_rotation_to(theta_desired)):
        if abs(l) > 0.003 and euclidean_error > 0.006:
            rospy.loginfo("Recalculating theta l: %f", abs(l))
            d_x = x_desired - x_current
            d_y = y_desired - y_current
            theta_desired = math.atan2(d_y, d_x)
        drive_forward()

        if(euclidean_error < 0.0015):
            rospy.loginfo('Target position reached')
            rospy.signal_shutdown('Target reached')
        else:
            rospy.loginfo("Moving forward euclidean distance: %f", euclidean_error)

def controller(msg):
    # Set once per point on trajectory
    set_init_pose()
    set_line_to_desired_position()

    # Loop
    log_pose()
    update_errors()
    update_pose_and_orientation(msg)
    go_to(x_desired, y_desired)

    # TODO: Reset and pick next point 

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
            left_motor_publisher.publish(motor_L)
            right_motor_publisher.publish(motor_R)
            rate.sleep()

    except rospy.ROSInterruptException:
        pass