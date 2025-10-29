#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
current=Twist(); target=Twist()
alpha=0.2
pub=None

def cb(msg):
    global target
    target=msg

if __name__=='__main__':
    rospy.init_node('velocity_ramp')
    rospy.Subscriber('/random_vel',Twist,cb)
    pub=rospy.Publisher('/cmd_vel',Twist,queue_size=10)
    rate=rospy.Rate(10)
    while not rospy.is_shutdown():
        current.linear.x += alpha*(target.linear.x - current.linear.x)
        current.angular.z += alpha*(target.angular.z - current.angular.z)
        pub.publish(current)
        rate.sleep()

