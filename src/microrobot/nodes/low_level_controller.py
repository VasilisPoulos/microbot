#!/usr/bin/env python

import rospy
import math
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from tf.transformations import euler_from_quaternion, quaternion_from_euler

left_motor_topic = "/microbot/left_joint_velocity_controller/command"
right_motor_topic = "/microbot/right_joint_velocity_controller/command"
odometry_topic = "/visual_odometry"
motor_L = 0
motor_R = 0
x_init = -1
y_init = -1
theta_init = -1
init_set = False
x_current = 0
y_current = 0
theta_current = 0

x_desired = 0.02
y_desired = 0.02
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

def on_site_rotation_to(theta_desired, accuracy=0.05):
    global motor_R, motor_L

    # P controller
    Kp_orientation = 600
    theta_error = theta_desired - theta_current
    omega_diff = Kp_orientation * abs((theta_error)) + motor_bias
    
    if theta_current > theta_desired:

        motor_L =   - omega_diff
        motor_R =   - omega_diff
    else:
        motor_L =   omega_diff
        motor_R =   omega_diff

    limiter(1450)

    if robot_reached_theta(theta_desired, theta_current, accuracy):
        rospy.loginfo('Target theta reached')
        #rospy.signal_shutdown('Target reached')
        return False
    else:
        rospy.loginfo("theta_desired: %f, theta_error: %f", math.degrees(theta_desired), \
            math.degrees(theta_desired - theta_current))
    return True

def set_init_pose():
    global x_init, y_init, init_set
    if not init_set:
        rospy.loginfo('Initialized pose x: %f y: %f theta:%f ', x_current, \
            y_current, math.degrees(theta_current))
        x_init = x_current
        y_init = y_current
        init_set = True

def reset_init_pose():
    global x_init, y_init, init_set
    x_init = -1
    y_init = -1
    init_set = False

def dist(p, q):
    return math.sqrt(sum((px - qx) ** 2.0 for px, qx in zip(p, q)))

def go_to(x_desired, y_desired, theta_desired, accuracy=0.05):
    global motor_R, motor_L

    # beta = theta_current + alpha - theta_desired
    alpha_target = math.atan2(y_desired, x_desired)
    direction = y_desired - y_current / x_desired - x_current
    alpha = 1
    beta = - direction
    gamma = - y_current - direction * x_current
    
    l = alpha*x_current + beta*y_current + gamma \
        / math.sqrt(alpha**2 + beta**2)

    acceptance_offset = 0.02
    Kp = 8000
    
    if not (on_site_rotation_to(alpha_target)):
        # rospy.loginfo("l: %f",  l)
        # diff = Kp*np.abs(l)
        # if l < 0 and l < - acceptance_offset:
        #     motor_R = 0
        #     motor_L = 1300 + diff
        # elif l > 0 and l > acceptance_offset:
        #     motor_R = 1300 + diff
        #     motor_L = 0
        rospy.loginfo("Moving forward euclidean distance: %f", \
            dist([x_current, y_current],[x_desired, y_desired]))
        motor_R = 1400
        motor_L = - 1400
        #limiter(1450)

def log_info():
    rospy.loginfo("rpmL: %f, rpmR: %f, x_current: %f, y_current: %f theta_current: %f", \
        motor_L, motor_R, x_current, y_current, 
        math.degrees(theta_current))

def controller(msg):
    update_pose_and_orientation(msg)
    set_init_pose()
    log_info()
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
            left_motor_publisher.publish(motor_L)
            right_motor_publisher.publish(motor_R)
            rate.sleep()

    except rospy.ROSInterruptException:
        pass