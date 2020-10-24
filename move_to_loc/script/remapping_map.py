#!/usr/bin/env python 

import rospy 
from nav_msgs.msg import OccupancyGrid


map_pub = None

def cb_map(msg) :
    global map_pub
    if map_pub != None:
        map_pub.publish(msg)

def main ():

    global map_pub

    rospy.init_node("remapping_node")
    rospy.Subscriber("/rtabmap/proj_map", OccupancyGrid, cb_map)
    map_pub = rospy.Publisher("/map", OccupancyGrid, queue_size=10)

    rospy.spin()

if __name__ == "__main__":

    main()