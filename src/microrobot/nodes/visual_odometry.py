#!/usr/bin/env python

# TODO: rename, visual odometry
import rospy
import math
from ar_track_alvar_msgs.msg import AlvarMarkers
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from tf.transformations import euler_from_quaternion, quaternion_from_euler
odometry_message = Odometry()

def alvar_callback(msg): 
    global odometry_message
    
    # Constructing odom message
    current_time = rospy.Time.now()
    odometry_message.header.stamp = current_time
    odometry_message.header.frame_id = "odom"
    odometry_message.child_frame_id = "base_link"

    marker_position = msg.markers[0].pose.pose.position
    x = - marker_position.y
    y = - marker_position.x
    z = marker_position.z

    marker_orientation = msg.markers[0].pose.pose.orientation
    orientation_list = [marker_orientation.x, marker_orientation.y, \
        marker_orientation.z, marker_orientation.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    
    fixed_yaw = - yaw - 1.57
    fixed_orientation = quaternion_from_euler(roll, pitch, fixed_yaw)
    # rospy.logwarn('deg: %f -> %f', math.degrees(yaw), \
    #     math.degrees(fixed_yaw))
    
    quartenion_x = fixed_orientation[0]
    quartenion_y = fixed_orientation[1]
    quartenion_z = fixed_orientation[2]
    quartenion_w = fixed_orientation[3]
    odometry_message.pose.pose = Pose(Point(x, y, z), 
        Quaternion(quartenion_x, quartenion_y, quartenion_z, quartenion_w))


    # TODO: I don't have speed information, but i could calculate it from the 
    # info above.
    odometry_message.twist.twist = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))

if __name__ == '__main__':
    try:
        rospy.init_node("visual_odometry", log_level=rospy.WARN)
        rate = rospy.Rate(10)
        rospy.loginfo('Starting visual odometry')
        
        # TODO: Change node to accept an arg for the robots name
        alvar_marker_sub = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, alvar_callback)
        visual_odometry_pub = rospy.Publisher("/visual_odometry", Odometry, queue_size=1)

        while not rospy.is_shutdown():
            visual_odometry_pub.publish(odometry_message)
            rate.sleep()

    except rospy.ROSInterruptException:
        # rospy.logwarn('Visual odometry failed')
        pass