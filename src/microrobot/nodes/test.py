#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry

left_motor_topic = '/microbot/left_joint_velocity_controller/command'
right_motor_topic = '/microbot/right_joint_velocity_controller/command'
gazebo_pose_topic = '/gazebo/model_states/pose[3]'

l_rpm = 1400.0
r_rpm = -1400.0
test_mode = 0
benchmark_0 = 0
benchmark_1 = 1
benchmark_2_drive = 0
benchmark_3_on_site = 0
controlled = 0

def robot_pose(msg):
    # rospy.loginfo('l_rpm: %f', msg.position.)
    pass

left_motor_publisher = rospy.Publisher(left_motor_topic, \
    Float64, queue_size=10)
right_motor_publisher = rospy.Publisher(right_motor_topic, \
    Float64, queue_size=10)   
# robot_pose_subscriber = rospy.Subscriber(gazebo_pose_topic, \
#     Odometry, robot_pose)

def reset_speed():
    left_motor_publisher.publish(0.0)
    right_motor_publisher.publish(0.0)
    rospy.loginfo('resetting speed...')

def drive(rpm, reverse = 1, time = 1):
    # reverse = 1 -> forward
    # reverse = -1 -> backwards
    l_rpm = -rpm*reverse
    r_rpm = rpm*reverse
    rospy.loginfo('l_rpm: %.1f r_rpm: %.1f', l_rpm, r_rpm)
    left_motor_publisher.publish(l_rpm)
    right_motor_publisher.publish(r_rpm + 50)
    # reset speed, sudden changes crash the sim
    # left_motor_publisher.publish(0.0)
    # right_motor_publisher.publish(0.0)
    # rospy.sleep(0.01)
    rospy.sleep(time)

def rotate(rpm, direction = 1, time=3):
    rospy.loginfo('l_rpm: %.1f r_rpm: %.1f', rpm*direction, \
         rpm*direction)
    left_motor_publisher.publish(rpm*direction)
    right_motor_publisher.publish(rpm*direction)
    rospy.sleep(time)

    # reset speed, sudden changes crash the sim
    left_motor_publisher.publish(0.0)
    right_motor_publisher.publish(0.0)
    rospy.sleep(0.01)

def turn(rpmR, rpmL, time=3):
    rospy.loginfo('l_rpm: %.1f r_rpm: %.1f', rpmR, rpmL)
    left_motor_publisher.publish(rpmL)
    right_motor_publisher.publish(rpmR)
    rospy.sleep(time)

    # reset speed, sudden changes crash the sim
    left_motor_publisher.publish(0.0)
    right_motor_publisher.publish(0.0)
    rospy.sleep(0.01)

def controlled_drive(direction=1, time=5):
    # go straight with a p controller.
    pass

if __name__ == '__main__':
    try:
        rospy.init_node("testing_node")
        rate = rospy.Rate(10)       
        rospy.on_shutdown(reset_speed)

        while not rospy.is_shutdown():
            rate.sleep()
            if test_mode:
                l_rpm = float(input("new_rpm:"))
                r_rpm = -l_rpm
                left_motor_publisher.publish(l_rpm)
                right_motor_publisher.publish(r_rpm)
                rate.sleep()
            elif benchmark_0:
                rpm_test_list = [1100, 1150, 1200, 1250, 1300, 1350, 1400, 1450, 1500]   
                for rpm in rpm_test_list:
                    drive(rpm, -1, 3)
                    rate.sleep()
                rospy.signal_shutdown('benchmark 0 done')
            elif benchmark_1:
                
                #drive(7640.0, 1, 20)
                #turn(1400.0, 1400.0, 30)
                rpm_test_list = [1150, 1200, 1250, 1300, 1350, 1400, 1450, 1500, 1550, 1600]
                for rpm in rpm_test_list:
                    turn(rpm, -1200, 20)
                    rate.sleep()

                rate.sleep()
                rospy.signal_shutdown('benchmark 1 done')
            elif benchmark_2_drive:
                rpm_test_list = [1150, 1200, 1250, 1300, 1350, 1400, 1450, 1500, 1550, 1600]
                for rpm in rpm_test_list:
                    drive(rpm, 1, 20)
                    rate.sleep()
                rospy.signal_shutdown('benchmark 2 done')
            elif benchmark_3_on_site:
                rpm_test_list = [1200, 1250, 1300, 1350, 1400, 1450, 1500, 1550, 1600]
                for rpm in rpm_test_list:
                    rotate(rpm, 1, 20)
                    rate.sleep()
                rospy.signal_shutdown('benchmark 3 done')

    except rospy.ROSInterruptException:
        pass