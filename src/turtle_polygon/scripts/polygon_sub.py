#!/usr/bin/env python3
import rospy, json, math
from geometry_msgs.msg import Twist
from std_msgs.msg import String

def move(pub, distance, speed=1.5):
    vel = Twist()
    vel.linear.x = speed
    duration = distance / speed
    rate = rospy.Rate(20)
    steps = int(duration * 20)
    for _ in range(steps):
        pub.publish(vel)
        rate.sleep()
    vel.linear.x = 0
    pub.publish(vel)

def turn(pub, angle_deg, ang_speed=90):
    vel = Twist()
    angular_speed = math.radians(ang_speed)
    duration = math.radians(angle_deg) / angular_speed
    vel.angular.z = angular_speed
    rate = rospy.Rate(20)
    steps = int(duration * 20)
    for _ in range(steps):
        pub.publish(vel)
        rate.sleep()
    vel.angular.z = 0
    pub.publish(vel)

def draw_square(pub, side):
    for _ in range(4):
        move(pub, side)
        turn(pub, 90)

def draw_rectangle(pub, l, b):
    for _ in range(2):
        move(pub, l)
        turn(pub, 90)
        move(pub, b)
        turn(pub, 90)

def draw_triangle(pub, side):
    for _ in range(3):
        move(pub, side)
        turn(pub, 120)

def draw_circle(pub, radius):
    vel = Twist()
    rate = rospy.Rate(50)
    vel.linear.x = 1.0
    vel.angular.z = 1.0 / radius
    steps = int((2 * math.pi * radius) / (vel.linear.x / 50))
    for _ in range(steps):
        pub.publish(vel)
        rate.sleep()
    vel.linear.x = 0
    vel.angular.z = 0
    pub.publish(vel)

def callback(msg):
    data = json.loads(msg.data)
    shape = data["shape"]
    rospy.loginfo(f"Drawing {shape}...")
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

    if shape == "square":
        draw_square(pub, data["side"])
    elif shape == "rectangle":
        draw_rectangle(pub, data["length"], data["breadth"])
    elif shape == "triangle":
        draw_triangle(pub, data["side"])
    elif shape == "circle":
        draw_circle(pub, data["radius"])

def polygon_subscriber():
    rospy.init_node('polygon_subscriber', anonymous=True)
    rospy.Subscriber('/polygon_cmd', String, callback)
    rospy.spin()

if __name__ == '__main__':
    polygon_subscriber()

