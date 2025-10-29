#!/usr/bin/env python3
import rospy, sys, termios, tty
from geometry_msgs.msg import Twist

class CustomTeleop:
    def __init__(self):
        rospy.init_node('custom_teleop')
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.speed_lin = 0.5
        self.speed_ang = 1.0
        self.msg = Twist()
        print("""
==============================
  Custom Teleop Controller
------------------------------
 w : forward
 s : backward
 a : turn left
 d : turn right
 space : stop
 Ctrl+C : quit
==============================
        """)
        self.loop()

    def get_key(self):
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            return sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)

    def loop(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            key = self.get_key()
            if key == 'w':
                self.msg.linear.x = self.speed_lin
                self.msg.angular.z = 0.0
            elif key == 's':
                self.msg.linear.x = -self.speed_lin
                self.msg.angular.z = 0.0
            elif key == 'a':
                self.msg.linear.x = 0.0
                self.msg.angular.z = self.speed_ang
            elif key == 'd':
                self.msg.linear.x = 0.0
                self.msg.angular.z = -self.speed_ang
            elif key == ' ':
                self.msg.linear.x = 0.0
                self.msg.angular.z = 0.0
            else:
                self.msg.linear.x = 0.0
                self.msg.angular.z = 0.0
            self.pub.publish(self.msg)
            rate.sleep()

if __name__ == "__main__":
    try:
        CustomTeleop()
    except rospy.ROSInterruptException:
        pass
    finally:
        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        stop = Twist()
        pub.publish(stop)
