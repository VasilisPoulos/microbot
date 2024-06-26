#!/usr/bin/env python

# TODO: rename, visual odometry
import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Pose2D
odometry_message = Odometry()
odometry_2d_message = Pose2D()
path = Path()

def alvar_callback(msg): 
    global odometry_message, path
    
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

    # 2D message
    odometry_2d_message.x = - marker_position.y
    odometry_2d_message.y = - marker_position.x
    odometry_2d_message.theta = yaw

    # Path
    path.header.frame_id = "odom"
    path.header.stamp = current_time

    pose = PoseStamped()
    pose.header.frame_id = "odom"
    pose.header.stamp = current_time
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0
    pose.pose.orientation.x = quartenion_x
    pose.pose.orientation.y = quartenion_y
    pose.pose.orientation.z = quartenion_z
    pose.pose.orientation.w = quartenion_w
    path.poses.append(pose)
    rospy.loginfo('%s', type(path.poses))
    path_pub.publish(path)

if __name__ == '__main__':
    try:
        rospy.init_node("visual_odometry", log_level=rospy.WARN)
        rate = rospy.Rate(10)
        rospy.loginfo('Starting visual odometry')
        
        # TODO: Change node to accept an arg for the robots name
        alvar_marker_sub = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, alvar_callback)
        visual_odometry_pub = rospy.Publisher("/visual_odometry", Odometry, queue_size=1)
        visual_2d_pub = rospy.Publisher("/visual_odometry_2d", Pose2D, queue_size=1)
        path_pub = rospy.Publisher('/path', Path, queue_size=10)

        while not rospy.is_shutdown():
            visual_odometry_pub.publish(odometry_message)
            visual_2d_pub.publish(odometry_2d_message)
            rate.sleep()

    except rospy.ROSInterruptException:
        # rospy.logwarn('Visual odometry failed')
        pass