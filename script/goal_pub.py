#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Odometry

from visualization_msgs.msg import MarkerArray, Marker

import numpy as np



goal = np.array([10.0, 10.0])


class Goal():
    def __init__(self):

        self.goal_pub = rospy.Publisher("goal", Pose2D, queue_size=10)
        self.marker_pub = rospy.Publisher("visual_goal", Marker, queue_size=10)

        self.odom_subscriber = rospy.Subscriber("goal_point", Odometry, self.goal_callback)

        self.goal_x = 0
        self.goal_y = 0

    def goal_callback(self, data):

        self.goal_x = data.pose.pose.position.x
        self.goal_y = data.pose.pose.position.y 

    def visualGoal(self):

        marker = Marker()
        marker.header.frame_id = "/odom"
        marker.header.stamp = rospy.Time.now()

        marker.type = marker.CUBE
        marker.action = marker.ADD
        marker.lifetime = rospy.Duration(0)

        marker.id = 0
        marker.ns = "basic_shapes"

        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.5

        marker.pose.position.x = self.goal_x
        marker.pose.position.y = self.goal_y
        marker.pose.position.z = 0
        marker.pose.orientation.x = 0
        marker.pose.orientation.y = 0
        marker.pose.orientation.z = 0
        marker.pose.orientation.w = 1

        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0

        self.marker_pub.publish(marker)
    
    def publish_goal(self):
        point = Pose2D()

        point.x = self.goal_x
        point.y = self.goal_y
        point.theta = 0

        self.goal_pub.publish(point)


if __name__ == "__main__":
    rospy.init_node("goal_pub")

    goal = Goal()


    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        goal.publish_goal()
        goal.visualGoal()
        rate.sleep()

    rospy.spin()