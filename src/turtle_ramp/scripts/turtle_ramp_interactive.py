#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import math

class TurtleRampInteractive:
    def __init__(self):
        rospy.init_node('turtle_ramp_interactive')
        self.pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(20)

        print("\n--- 🐢 Turtlesim Velocity Ramp Interactive ---")
        self.shape = input("Enter shape (square/rectangle/triangle/circle): ").strip().lower()
        self.length = float(input("Enter side length or radius: "))
        self.ramp_rate = float(input("Enter ramp acceleration (0.1–2.0 recommended): "))
        self.max_speed = float(input("Enter max linear speed (0.5–3.0 recommended): "))

        self.max_lin = self.max_speed
        self.max_ang = 2.0
        self.acc_lin = self.ramp_rate
        self.acc_ang = self.ramp_rate * 2
        self.curr_lin = 0.0
        self.curr_ang = 0.0

    def ramp(self, curr, target, accel, dt):
        diff = target - curr
        step = accel * dt
        if abs(diff) < step:
            return target
        return curr + math.copysign(step, diff)

    def move(self, lin, ang, duration):
        end_time = rospy.Time.now() + rospy.Duration(duration)
        while rospy.Time.now() < end_time and not rospy.is_shutdown():
            dt = 1.0 / 20.0
            self.curr_lin = self.ramp(self.curr_lin, lin, self.acc_lin, dt)
            self.curr_ang = self.ramp(self.curr_ang, ang, self.acc_ang, dt)
            twist = Twist()
            twist.linear.x = self.curr_lin
            twist.angular.z = self.curr_ang
            self.pub.publish(twist)
            self.rate.sleep()
        self.stop()

    def stop(self):
        twist = Twist()
        self.pub.publish(twist)
        rospy.sleep(0.5)

    def draw_square(self):
        for _ in range(4):
            self.move(self.max_lin, 0.0, self.length / self.max_lin)
            self.move(0.0, self.max_ang, math.pi / 2 / self.max_ang)

    def draw_rectangle(self):
        width = float(input("Enter rectangle width: "))
        for _ in range(2):
            self.move(self.max_lin, 0.0, self.length / self.max_lin)
            self.move(0.0, self.max_ang, math.pi / 2 / self.max_ang)
            self.move(self.max_lin, 0.0, width / self.max_lin)
            self.move(0.0, self.max_ang, math.pi / 2 / self.max_ang)

    def draw_triangle(self):
        for _ in range(3):
            self.move(self.max_lin, 0.0, self.length / self.max_lin)
            self.move(0.0, self.max_ang, 2 * math.pi / 3 / self.max_ang)

    def draw_circle(self):
        time = 2 * math.pi * self.length / self.max_lin
        self.move(self.max_lin, self.max_lin / self.length, time)

    def run(self):
        rospy.sleep(1.0)
        if self.shape == "square":
            self.draw_square()
        elif self.shape == "rectangle":
            self.draw_rectangle()
        elif self.shape == "triangle":
            self.draw_triangle()
        elif self.shape == "circle":
            self.draw_circle()
        else:
            print("Unknown shape!")
        self.stop()
        print("\n✅ Done drawing with velocity ramps!\n")

if __name__ == "__main__":
    try:
        node = TurtleRampInteractive()
        node.run()
    except rospy.ROSInterruptException:
        pass
