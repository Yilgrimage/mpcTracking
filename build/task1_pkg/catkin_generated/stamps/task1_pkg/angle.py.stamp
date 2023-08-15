import rospy
from geometry_msgs.msg import PointStamped,Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from math import atan2, sqrt

def odomcallback(odom):
    (roll, pitch, yaw) = euler_from_quaternion([odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w])
    rospy.loginfo("yaw: %f",yaw)

rospy.init_node("turtle_angle")
sub = rospy.Subscriber("/odom",Odometry,odomcallback,queue_size=10)

rospy.spin()