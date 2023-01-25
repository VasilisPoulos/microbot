#!/usr/bin/env python

import rospy
import math
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from std_msgs.msg import Float64
from tf.transformations import euler_from_quaternion

import sys
sys.path.insert(1, '/home/vasilisp/catkin_ws/src/microrobot/src/microrobot/scripts/')

import numpy as np
import new_cloth 

# Ros topics
# Existing topics
left_motor_topic = "/microbot/left_joint_velocity_controller/command"
right_motor_topic = "/microbot/right_joint_velocity_controller/command"
odometry_topic = "/visual_odometry"

# Creating and publishing Marker
marker_topic = "/goal_visualization"
marker = Marker()

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
# Linear equation variables for line (l) that connects current position to 
# desired
point1 = [0.02, -0.03]
point2 = [0.02, 0.01]
point3 = [-0.01, 0.01]
point4 = [-0.03, -0.04]
points_list = [point1, point2, point3, point4]

s1 = np.array([0., 0., 1])
p = np.array([0.05, 0., 1.])
s2 = np.array([0.025, 0.05, 1.])
# TODO: catch 180 deg case in a function that handles the user input.
c1 = new_cloth.Clothoid(s1, p, s2, max_dev=0.002, num_of_points=3)
clothoids = c1.path 
already_outside = False
num_of_point = 0

alpha = 1
beta = 0
gamma = 0
equation_is_set = False
x_desired = clothoids[0][0]
y_desired = clothoids[0][1]   #TODO: fix division by zero error
theta_desired = 0

# Error
euclidean_error = 0
theta_error = 0

# Biases per type of movement
# On site
Kp_on_site = 200        # P gain for on site rotation
on_site_bias = 1300     # min motor speed for on site rotation
on_site_limit = 1350    # max motor speed to reduce bouncing
on_site_accuracy = 0.02 # Acceptance offset in rad 
# Drive forward
Kp_drive = 2800         # P gain for drive forward
drive_bias = 1200       # min motor speed for drive forward
drive_limit = 1250      # max motor speed to reduce bouncing
drift_limit = 0.003     # maximum distance allowed to drift away from line (l)
goal_offset = 0.01      # used to stop theta correction when reaching goal
goal_accuracy = 0.0018  # acceptable distance from desired point

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

def drive_forward():
    global motor_L, motor_R
    
    # P controller
    rpm = Kp_drive*euclidean_error + drive_bias
    motor_L = - rpm
    motor_R = rpm
    limiter(drive_limit)

def turn_left(rpm):
    global motor_L, motor_R
    motor_L =   rpm
    motor_R =   rpm

def turn_right(rpm):
    global motor_L, motor_R
    motor_L =   - rpm
    motor_R =   - rpm

def on_site_rotation_to(theta_desired, accuracy=0.02):
    global motor_R, motor_L
        
    if robot_reached_theta(theta_desired, theta_current, accuracy):
        return False
    else:
        # P controller
        rpm = Kp_on_site * abs((theta_error)) + on_site_bias
        
        # theta_desired refers to the desired direction of the robot with respect
        # to the odom axis, it is normalized to [-pi, pi]. A new range is needed,
        # that is calculated with starting position the robot's theta_current, to 
        # avoid unwanted behavior when the robot's direction is pi. Slight slippage
        # or a recalculation fo the desired theta to -pi may make the controller 
        # think that the robot needs to make a full right hand rotation to -pi, even 
        # though the left hand path is shorter.

        # if theta_current >= 0:
        #     opposite_to_th_current = -math.pi + theta_current
        #     if opposite_to_th_current >= theta_desired >= - math.pi \
        #         or math.pi > theta_desired > theta_current:           
        #         # Theta desired is to the left of theta_current
        #         rospy.loginfo('Turning LEFT to %.2f [%.2f, %.2f]', 
        #             math.degrees(theta_desired), opposite_to_th_current, theta_current)
        #         turn_left(rpm)
        #     elif theta_current > theta_desired >= 0 \
        #         or 0 > theta_desired > opposite_to_th_current:
        #         # Theta desired is to the right of theta_current
        #         rospy.loginfo('Turning RIGHT to %.2f [%.2f, %.2f]', 
        #             math.degrees(theta_desired), opposite_to_th_current, theta_current)
        #         turn_right(rpm)
        # elif theta_current < 0:
        #     opposite_to_th_current = math.pi + theta_current
        #     if theta_current > theta_desired > - math.pi \
        #         or math.pi >= theta_desired >= opposite_to_th_current:           
        #         # Theta desired is to the right of theta_current
        #         rospy.loginfo('Turning RIGHT to %.2f [%.2f, %.2f]', 
        #             math.degrees(theta_desired), opposite_to_th_current, theta_current)
        #         turn_right(rpm)
        #     elif 0 <= theta_desired < opposite_to_th_current \
        #         or 0 > theta_desired > theta_current:
        #         # Theta desired is to the left of theta_current
        #         rospy.loginfo('Turning LEFT to %.2f [%.2f, %.2f]', 
        #             math.degrees(theta_desired), opposite_to_th_current, theta_current)
        #         turn_left(rpm)

        if theta_current > theta_desired:
            turn_right(rpm)
        else:
            turn_left(rpm)

        limiter(on_site_limit)

        rospy.loginfo("Theta_error: %f", math.degrees(theta_error))
    return True

def goal_marker_update():
    marker.header.frame_id = "odom"
    marker.header.stamp = rospy.Time.now()
    # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
    marker.type = 2
    marker.id = 0
    # Set the scale of the marker
    marker.scale.x = 0.005
    marker.scale.y = 0.005
    marker.scale.z = 0.005
    # Set the color
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.color.a = 1.0
    # Set the pose of the marker
    marker.pose.position.x = x_desired
    marker.pose.position.y = y_desired
    marker.pose.position.z = 0
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0

def go_to():
    global motor_R, motor_L, alpha, beta, gamma, theta_desired, x_desired, y_desired, num_of_point, \
    equation_is_set, already_outside
    # Calculate distance from line (l) to catch the robot drifting away
    distance_from_line = \
        alpha*x_current + beta*y_current + gamma / math.sqrt(alpha**2 + beta**2)

    # First rotate to point
    if not (on_site_rotation_to(theta_desired, accuracy=on_site_accuracy)):
        # then drive to desired position, fix theta_error on the way
        drive_forward()
        # If robot drifts away form l, recalculate theta desired to correct 
        # it's direction. If it's too close to the desired point, skip the
        # correction.
        if abs(distance_from_line) > drift_limit \
        and euclidean_error > goal_offset and not already_outside:
            rospy.loginfo("[WARNING] Recalculating theta l: %f", abs(distance_from_line))
            d_x = x_desired - x_current
            d_y = y_desired - y_current
            theta_desired = math.atan2(d_y, d_x)
            already_outside = True 
        elif abs(distance_from_line) < drift_limit:
            already_outside = False

        if(euclidean_error < goal_accuracy):
            rospy.loginfo('Target position reached')
            x_desired = clothoids[num_of_point][0]
            y_desired = clothoids[num_of_point][1]
            num_of_point += 1
            reset_init_pose()
            set_init_pose()
            equation_is_set = False
            if num_of_point == len(clothoids):
                rospy.signal_shutdown('Target reached')
        else:
            rospy.loginfo("Moving forward, euclidean distance: %f", \
                euclidean_error)

def controller(msg):
    # Set once per point on trajectory
    set_init_pose()
    set_line_to_desired_position()

    # Loop
    goal_marker_update()
    log_pose()
    update_errors()
    update_pose_and_orientation(msg)
    go_to()

if __name__ == '__main__':
    try: 
        rospy.init_node("low_level_controller")
        rospy.loginfo('Starting.')
        rate = rospy.Rate(10)
        rospy.on_shutdown(stop_motors)
        
        left_motor_publisher = rospy.Publisher(left_motor_topic, Float64, queue_size=1)
        right_motor_publisher = rospy.Publisher(right_motor_topic, Float64, queue_size=1)
        odometry_subscriber = rospy.Subscriber(odometry_topic, Odometry, controller)
        marker_pub = rospy.Publisher(marker_topic, Marker, queue_size = 2)

        while not rospy.is_shutdown():
            left_motor_publisher.publish(motor_L)
            right_motor_publisher.publish(motor_R)
            marker_pub.publish(marker)
            rate.sleep()

    except rospy.ROSInterruptException:
        pass