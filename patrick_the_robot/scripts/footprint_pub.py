#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PolygonStamped, Polygon

footprint = Polygon()
received  = False

def callback(data):
    global footprint
    global received
    footprint = data.polygon
    received = True

def footprint_publisher():
    global footprint
    global received
    pub = rospy.Publisher('/move_base/local_costmap/footprint', Polygon, queue_size=50)
    rospy.init_node('footprint_publisher', anonymous=True)
    rate = rospy.Rate(10)
    rospy.Subscriber('/move_base/local_costmap/obstacle_layer_footprint/footprint_stamped', PolygonStamped, callback)
    while not rospy.is_shutdown():
        if received:
            pub.publish(footprint)
            received = False
        rate.sleep()
        
if __name__ == '__main__':
    try:
        footprint_publisher()
    except rospy.ROSInterruptException:
        pass
