#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PointStamped,Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from math import atan2, sqrt

class pid():
    def __init__(self,p,i,d,limit = 10):
        self.Kp = p
        self.Ki = i
        self.Kd = d
        self.integration = 0
        self.last_error = 0
        self.output_limit = limit

    def pid_func(self,error):
        self.integration += self.Ki * error
        output = self.Kp * error + self.Ki * self.integration + self.Kd * (error - self.last_error)
        self.last_error = error
        if(output < self.output_limit):
            return output
        else:
            return self.output_limit
        
class Turtle3Control(pid):
    def __init__(self):
        rospy.init_node("turtle_nav")
        self.goal_pos = PointStamped()
        self.current_pos = Odometry()
        self.vel_pub = rospy.Publisher("/cmd_vel",Twist,queue_size=10)
        self.current_pos_sub = rospy.Subscriber("/odom",Odometry,self.current_point_callback,queue_size=10)
        self.goal_pos_sub = rospy.Subscriber('/target_point',PointStamped,self.goal_point_callback,queue_size=10)
        self.rate = rospy.Rate(200)

        self.angle_pid = pid(0.4,0,0.1,limit=1)
        self.linear_pid = pid(0.8,0,0.0,limit = 0.6)

    def current_point_callback(self,pos):
        self.current_pos = pos
        self.updatecommandVelocity()

    def goal_point_callback(self,pos):
        self.goal_pos = pos
        self.updatecommandVelocity()

    def updatecommandVelocity(self):
        cmd_vel = Twist()
        self.current_pos.pose.pose.orientation.w
        (roll, pitch, yaw) = euler_from_quaternion([self.current_pos.pose.pose.orientation.x, self.current_pos.pose.pose.orientation.y, self.current_pos.pose.pose.orientation.z, self.current_pos.pose.pose.orientation.w])
        if(yaw < 0):
            yaw += 2*3.1415926
        goal_angle = atan2(self.goal_pos.point.y - self.current_pos.pose.pose.position.y,
                           self.goal_pos.point.x - self.current_pos.pose.pose.position.x)
        if(goal_angle < 0):
            goal_angle += 2*3.1415926
        angle_error = goal_angle - yaw
        cmd_vel.angular.z = self.angle_pid.pid_func(angle_error)
        if(abs(angle_error) < 0.6):
            distance = sqrt((self.goal_pos.point.x - self.current_pos.pose.pose.position.x)**2 +
                (self.goal_pos.point.y - self.current_pos.pose.pose.position.y)**2)
            cmd_vel.linear.x = self.linear_pid.pid_func(distance)
        else:
            cmd_vel.linear.x = 0
        self.vel_pub.publish(cmd_vel)
    

turtle = Turtle3Control()

while not rospy.is_shutdown():
    #turtle.updatecommandVelocity()
    rospy.spin()

