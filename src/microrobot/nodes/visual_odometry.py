#!/usr/bin/env python

# TODO: rename, visual odometry
import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
odometry_message = Odometry()

def alvar_callback(msg): 
    global odometry_message
    
    x = - msg.markers[0].pose.pose.position.y
    y = - msg.markers[0].pose.pose.position.x
    z = msg.markers[0].pose.pose.position.z
    quartenion_x = msg.markers[0].pose.pose.orientation.w
    quartenion_y = - msg.markers[0].pose.pose.orientation.z
    quartenion_z = - msg.markers[0].pose.pose.orientation.y
    quartenion_w = - msg.markers[0].pose.pose.orientation.x
    
    current_time = rospy.Time.now()
    odometry_message.header.stamp = current_time
    odometry_message.header.frame_id = "odom"
    odometry_message.child_frame_id = "base_link"
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