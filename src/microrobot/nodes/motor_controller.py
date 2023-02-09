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
pose_is_set = False

# Current position
x_current = 0
y_current = 0
theta_current = 0

# Goal
point1 = [0.02, -0.03]
point2 = [0.02, 0.01]
point3 = [-0.01, 0.01]
point4 = [-0.03, -0.04]
points_list = [point1, point2, point3, point4]

p = np.array([0.032, 0.025, 1.])
s2 = np.array([0.045, 0.06, 1.])
# TODO: catch 180 deg case in a function that handles the user input.
clothoids = new_cloth.calculate_trajectory(p, s2, 0.0015, 4, 2)

#[[0.02, -0.03], [0.02, -0.04]]


# Linear equation variables for line (l) that connects current position to 
# desired
inside_bounds = True
num_of_point = 0
alpha = -1
beta = -1
gamma = -1
line_to_destination_is_set = False
x_desired = clothoids[0][0]
y_desired = clothoids[0][1]   #TODO: fix division by zero error
theta_desired = 0

# Error
euclidean_error = 0
theta_error = 0

# Biases per type of movement
# On site
Kp_on_site = 100        # P gain for on site rotation
on_site_bias = 1305     # min motor speed for on site rotation
on_site_limit = 1335    # max motor speed to reduce bouncing
on_site_accuracy = 0.04 # Acceptance offset in rad 
# Drive forward
Kp_drive = 2050         # P gain for drive forward
drive_bias = 1340       # min motor speed for drive forward
drive_limit = 1360      # max motor speed to reduce bouncing
drift_limit = 0.0012    # maximum distance allowed to drift away from line (l)
goal_offset = 0.0035     # used to stop theta correction when reaching goal
goal_accuracy = 0.0018  # acceptable distance from desired point

# Utility functions
def set_pose():
    global x_init, y_init, pose_is_set
    if not pose_is_set:
        x_init = x_current
        y_init = y_current
        pose_is_set = True
        rospy.loginfo('Initialized pose x: %f y: %f theta:%f ', x_current, \
            y_current, math.degrees(theta_current))

def reset_pose():
    global x_init, y_init, pose_is_set
    x_init = -1
    y_init = -1
    pose_is_set = False

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
    global theta_desired, alpha, beta, gamma, line_to_destination_is_set

    if not line_to_destination_is_set:
        d_x = x_desired - x_init
        d_y = y_desired - y_init
        theta_desired = math.atan2(d_y, d_x)
        rospy.loginfo('from (%f, %f) to  (%f, %f)', x_init, y_init, x_desired, y_desired)
        direction = d_y / d_x
        alpha = direction
        beta =  -1
        gamma = - x_init*direction + y_init
        rospy.loginfo('Initialized line trajectory a: %f b: %f c:%f direction: %f', \
                      alpha, beta, gamma, direction)
        
        check1 = alpha * x_desired + beta* y_desired + gamma
        check2 = alpha * x_init+ beta* y_init + gamma
        if  round(check1, 6) == 0.0 and round(check2, 6) == 0.0:
            rospy.loginfo("Equation is correctly set")
        line_to_destination_is_set = True

def min_distance_from_line_trajectory():
    # @https://stackoverflow.com/questions/39840030/distance-between-point-and-a-line-from-two-points
    p1=np.array([x_init,y_init])
    p2=np.array([x_desired,y_desired])
    p3=np.array([x_current,y_current])
    d=np.cross(p2-p1,p3-p1)/np.linalg.norm(p2-p1)
    # alpha*x_current + beta*y_current + gamma / math.sqrt(alpha**2 + beta**2)
    return d

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
        
    # if robot_reached_theta(theta_desired, theta_current, accuracy):
    #     return False
    # else:
        # P controller
    rpm = Kp_on_site * abs((theta_error)) + on_site_bias
    
    if theta_current > theta_desired:
        turn_right(rpm)
    else:
        turn_left(rpm)

    limiter(on_site_limit)
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

    #return True
    rospy.loginfo("Theta_error: %f", math.degrees(theta_error))
  
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

# Bool functions 

def robot_is_between_bounds():
    distance_from_line_trajectory = min_distance_from_line_trajectory()
    if abs(distance_from_line_trajectory) > drift_limit \
        and euclidean_error > goal_offset: 
        return True
    return False

def recalculate_theta_des():
    d_x = x_desired - x_current
    d_y = y_desired - y_current
    return  math.atan2(d_y, d_x)

def target_reached():
    return euclidean_error < goal_accuracy

# Control functions 

def go_to_desired_position():
    global theta_desired, x_desired, y_desired, num_of_point, line_to_destination_is_set

    l = min_distance_from_line_trajectory()
    if robot_is_between_bounds():
            rospy.loginfo("Out of bounds, distance: %f", l - drift_limit)
            theta_desired = recalculate_theta_des()
    
    if not robot_reached_theta(theta_desired, theta_current, on_site_accuracy):
        if euclidean_error < goal_offset:
            rospy.loginfo("On site deactivated")
        else:
            rospy.loginfo("Turning %f rad, %f deg", theta_desired, math.degrees(theta_desired))
            on_site_rotation_to(theta_desired)
    else:
        rospy.loginfo("Driving")
        drive_forward()

    if target_reached():
        num_of_point += 1
        if num_of_point == len(clothoids):
            rospy.signal_shutdown('Target reached')
        else:            
            x_desired = clothoids[num_of_point][0]
            y_desired = clothoids[num_of_point][1]
            
            reset_pose()
            set_pose()
            line_to_destination_is_set = False
            set_line_to_desired_position()      

def controller(msg):
    
    goal_marker_update()
    log_pose()
    update_errors()
    update_pose_and_orientation(msg)

    set_pose()
    set_line_to_desired_position()
    go_to_desired_position()

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