#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import json
import matplotlib.pyplot as plt
import numpy as np

def draw_shape(data):
    shape = data["shape"]
    plt.clf()

    if shape == "square":
        s = data["side"]
        x = [0, s, s, 0, 0]
        y = [0, 0, s, s, 0]
    elif shape == "rectangle":
        l = data["length"]
        b = data["breadth"]
        x = [0, l, l, 0, 0]
        y = [0, 0, b, b, 0]
    elif shape == "triangle":
        s = data["side"]
        h = (np.sqrt(3)/2)*s
        x = [0, s/2, s, 0]
        y = [0, h, 0, 0]
    elif shape == "circle":
        r = data["radius"]
        theta = np.linspace(0, 2*np.pi, 100)
        x = r*np.cos(theta)
        y = r*np.sin(theta)
    else:
        return

    plt.plot(x, y, 'b-', linewidth=2)
    plt.axis('equal')
    plt.title(f"{shape.capitalize()} Visualization")
    plt.pause(0.01)

def callback(msg):
    data = json.loads(msg.data)
    rospy.loginfo(f"Received: {data}")
    draw_shape(data)

def polygon_subscriber():
    rospy.init_node('polygon_subscriber', anonymous=True)
    rospy.Subscriber('/polygon_data', String, callback)
    plt.ion()
    plt.show()
    rospy.spin()

if __name__ == '__main__':
    polygon_subscriber()
