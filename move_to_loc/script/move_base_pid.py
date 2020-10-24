#!/usr/bin/env python

import rospy 
import math
import tf
import geometry_msgs.msg


GOAL = geometry_msgs.msg.PointStamped()
CURRENT = geometry_msgs.msg.PointStamped()
VIPER_IN_MAP_TRANSLATION = None


def cb_aruco(msg):

    global GOAL, CURRENT, VIPER_IN_MAP_TRANSLATION

    if VIPER_IN_MAP_TRANSLATION != None :
        GOAL.header = msg.header
        GOAL.point.x = VIPER_IN_MAP_TRANSLATION[0] + msg.point.x
        GOAL.point.y = VIPER_IN_MAP_TRANSLATION[0] + msg.point.y
        GOAL.point.z = VIPER_IN_MAP_TRANSLATION[0] + msg.point.z

    rospy.loginfo(GOAL)
    
def main():

    global GOAL, CURRENT, VIPER_IN_MAP_TRANSLATION

    rospy.init_node('turtle_tf_listener')
    listener = tf.TransformListener()

    rospy.Subscriber("/VIPER/follow_aruco/target_position", geometry_msgs.msg.PointStamped, cb_aruco)

    # turtle_vel = rospy.Publisher('turtle2/cmd_vel', geometry_msgs.msg.Twist,queue_size=1)
    CONTROL_HZ = 10.0

    rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/map', '/VIPER/left_camera_optical_frame_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        
        VIPER_IN_MAP_TRANSLATION = trans
        # rospy.loginfo(trans)
        # rospy.loginfo(GOAL)
        # rospy.loginfo(trans)
        # angular = 4 * math.atan2(trans[1], trans[0])
        # linear = 0.5 * math.sqrt(trans[0] ** 2 + trans[1] ** 2)
        # cmd = geometry_msgs.msg.Twist()
        # cmd.linear.x = linear
        # cmd.angular.z = angular

        rate.sleep()

    rospy.spin()

if __name__ == '__main__':

    main()