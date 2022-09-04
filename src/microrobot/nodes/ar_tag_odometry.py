#!/usr/bin/env python

# TODO: rename, visual odometry
import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
odometry_message = Odometry()

def alvar_callback(msg): 
    global odometry_message
    
    x = -msg.markers[0].pose.pose.position.x
    y = msg.markers[0].pose.pose.position.y
    current_time = rospy.Time.now()
    odometry_message.header.stamp = current_time
    odometry_message.header.frame_id = "odom"
    odometry_message.child_frame_id = "base_link"
    odometry_message.pose.pose = Pose(Point(x, y, 0), Quaternion(0, 0, 0, 0))
    odometry_message.twist.twist = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))

if __name__ == '__main__':
    try:
        rospy.init_node("visual_odometry")
        rate = rospy.Rate(10)
        rospy.loginfo('Starting visual odometry')
        
        alvar_marker_sub = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, alvar_callback)
        visual_odometry_pub = rospy.Publisher("/visual_odometry", Odometry, queue_size=1)

        while not rospy.is_shutdown():
            visual_odometry_pub.publish(odometry_message)
            rate.sleep()

    except rospy.ROSInterruptException:
        pass