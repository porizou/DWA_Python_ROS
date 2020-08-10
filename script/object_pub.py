#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

from std_msgs.msg import Float32MultiArray

from visualization_msgs.msg import MarkerArray, Marker

import numpy as np


# obstacles [x(m) y(m), ....]
"""
ob = np.array([[-1, -1],
                [0, 2],
                [4.0, 2.0],
                [5.0, 4.0],
                [5.0, 5.0],
                [5.0, 6.0],
                [5.0, 9.0],
                [8.0, 9.0],
                [7.0, 9.0],
                [8.0, 10.0],
                [9.0, 11.0],
                [12.0, 13.0],
                [12.0, 12.0],
                [15.0, 15.0],
                [13.0, 13.0]
                ])
"""
ob = np.array([[3.38, 0.22],
                [3.3, 0.08],
                [3.3, -0.31],
                [3.36, -0.67],
                [3.37, -1.08],
                [3.37, -0.52]
                ])

        

def visual_object_pub(object):


    markers = MarkerArray() 
    for m in range(object.shape[0]):
        marker = Marker()

        marker.header.frame_id = "/odom"
        marker.header.stamp = rospy.Time.now()

        marker.type = marker.CYLINDER
        marker.action = marker.ADD
        marker.lifetime = rospy.Duration(0)

        marker.id = m
        marker.ns = "basic_shapes"

        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.5

        marker.pose.position.x = object[m, 0]
        marker.pose.position.y = object[m, 1]
        marker.pose.position.z = 0
        marker.pose.orientation.x = 0
        marker.pose.orientation.y = 0
        marker.pose.orientation.z = 0
        marker.pose.orientation.w = 1

        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        markers.markers.append(marker)
    
    marker_pub.publish(markers)






if __name__ == "__main__":
    rospy.init_node("publish_object_point")
    pub = rospy.Publisher("object_points", Float32MultiArray, queue_size=10)

    marker_pub = rospy.Publisher("visual_object", MarkerArray, queue_size=10)

    rate = rospy.Rate(2)
    while not rospy.is_shutdown():

        print(ob.shape)
        point = Float32MultiArray()

        point.data = ob.flatten()

        pub.publish(point)

        visual_object_pub(ob)

        rate.sleep()

    rospy.spin()


