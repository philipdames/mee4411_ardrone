#!/usr/bin/env python

import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

XMIN = 0
YMIN = 1
ZMIN = 2
XMAX = 3
YMAX = 4
ZMAX = 5
RED = 6
GREEN = 7
BLUE= 8

def blockVis3d():
    rospy.init_node('block_vis')
    
    blocksPub = rospy.Publisher('env_markers', MarkerArray, queue_size=1, latch=True)

    # read in map parameters
    boundary = rospy.get_param('~boundary')
    blocks = rospy.get_param('~blocks')
    frame_id = rospy.get_param('~frame_id', '/map')
    alpha = rospy.get_param('~alpha', 0.5)
    
    # initialize header and color
    h = Header()
    h.frame_id = frame_id
    h.stamp = rospy.Time.now()
    
    # initialize block marker message
    ma = [] # marker array
    i = 0
    for b in blocks:
        bm = Marker();
        bm.header = h
        bm.ns = "blocks"
        bm.id = i
        bm.type = Marker.CUBE
        bm.action = Marker.ADD
        bm.pose.position.x = (b[XMIN]+b[XMAX])/2.
        bm.pose.position.y = (b[YMIN]+b[YMAX])/2.
        bm.pose.position.z = (b[ZMIN]+b[ZMAX])/2.
        bm.scale.x = b[XMAX] - b[XMIN]
        bm.scale.y = b[YMAX] - b[YMIN]
        bm.scale.z = b[ZMAX] - b[ZMIN]
        bm.color.r = b[RED]/255.
        bm.color.g = b[GREEN]/255.
        bm.color.b = b[BLUE]/255.
        bm.color.a = alpha # set transparency
        
        ma.append(bm)
        i = i + 1
    
    blocksPub.publish(MarkerArray(markers=ma))
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    try:
        blockVis3d()
    except rospy.ROSInterruptException:
        pass

