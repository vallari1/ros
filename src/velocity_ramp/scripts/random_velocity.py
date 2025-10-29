#!/usr/bin/env python3
import rospy,random
from geometry_msgs.msg import Twist
pub=rospy.Publisher('/random_vel',Twist,queue_size=10)
rospy.init_node('random_velocity')
r=rospy.Rate(1)
while not rospy.is_shutdown():
    t=Twist()
    t.linear.x=random.uniform(-1,1)
    t.angular.z=random.uniform(-2,2)
    pub.publish(t); r.sleep()

