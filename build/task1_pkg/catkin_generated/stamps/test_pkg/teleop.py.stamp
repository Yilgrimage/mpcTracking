#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PointStamped
import sys

def input_point():
    try:
        x = float(input("Enter x coordinate of the target point: "))
        y = float(input("Enter y coordinate of the target point: "))
        return x, y, 0
    except ValueError:
        print("Invalid input! Please enter numeric values.")
        return None

def main():
    rospy.init_node('set_target_point', anonymous=True)
    target_point_publisher = rospy.Publisher('/target_point', PointStamped, queue_size=10)

    while not rospy.is_shutdown():
        target_point = PointStamped()
        target_point.header.stamp = rospy.Time.now()
        target_point.header.frame_id = "map"

        point = input_point()
        if point is not None:
            target_point.point.x, target_point.point.y, target_point.point.z = point
            target_point_publisher.publish(target_point)

        rospy.sleep(1.0)

main()
