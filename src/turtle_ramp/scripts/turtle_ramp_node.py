#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import math

class TurtleRamp:
    def __init__(self):
        rospy.init_node('turtle_ramp_node')
        self.pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(20)
        self.max_lin = rospy.get_param("~max_linear_speed", 2.0)
        self.max_ang = rospy.get_param("~max_angular_speed", 2.0)
        self.acc_lin = rospy.get_param("~linear_acc", 0.5)
        self.acc_ang = rospy.get_param("~angular_acc", 1.0)
        self.target_lin = 0.0
        self.target_ang = 0.0
        self.curr_lin = 0.0
        self.curr_ang = 0.0

    def ramp(self, curr, target, accel, dt):
        diff = target - curr
        step = accel * dt
        if abs(diff) < step:
            return target
        return curr + math.copysign(step, diff)

    def move_pattern(self):
        seq = [
            (1.0, 0.0, 2.0),   # forward
            (0.0, 1.0, 1.0),   # turn
            (1.0, 0.0, 2.0),
            (0.0, 1.0, 1.0),
            (1.0, 0.0, 2.0),
            (0.0, 1.0, 1.0),
            (1.0, 0.0, 2.0),
            (0.0, 1.0, 1.0),
        ]
        for lin, ang, t in seq:
            self.target_lin = lin * self.max_lin
            self.target_ang = ang * self.max_ang
            end_time = rospy.Time.now() + rospy.Duration(t)
            while rospy.Time.now() < end_time and not rospy.is_shutdown():
                dt = 1.0 / 20.0
                self.curr_lin = self.ramp(self.curr_lin, self.target_lin, self.acc_lin, dt)
                self.curr_ang = self.ramp(self.curr_ang, self.target_ang, self.acc_ang, dt)
                twist = Twist()
                twist.linear.x = self.curr_lin
                twist.angular.z = self.curr_ang
                self.pub.publish(twist)
                self.rate.sleep()
        self.stop()

    def stop(self):
        twist = Twist()
        self.pub.publish(twist)

if __name__ == "__main__":
    node = TurtleRamp()
    rospy.sleep(1.0)
    node.move_pattern()
