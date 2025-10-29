#!/usr/bin/env python3
import rospy, random, math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np

class WanderBotSafety:
    def __init__(self):
        rospy.init_node('wander_bot_safety')
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        self.min_safe_distance = rospy.get_param('~min_safe_distance', 0.4)
        self.max_linear_speed = rospy.get_param('~max_linear_speed', 0.25)
        self.max_angular_speed = rospy.get_param('~max_angular_speed', 1.0)
        self.fov = rospy.get_param('~fov', 60)  # frontal sector degrees
        self.rate = rospy.Rate(rospy.get_param('~rate', 10))

        self.front_distance = float('inf')
        self.no_scan_data = True
        rospy.logwarn("No /scan detected — using simulated data for testing.")

        while not rospy.is_shutdown():
            if self.no_scan_data:
                self.front_distance = random.uniform(0.2, 2.5)
            self.move()
            self.rate.sleep()

    def scan_callback(self, msg):
        n = len(msg.ranges)
        mid = n // 2
        spread = int((self.fov / 360.0) * n)
        sector = msg.ranges[mid - spread:mid + spread]
        clean = [r for r in sector if not math.isinf(r)]
        if clean:
            self.front_distance = min(clean)
            self.no_scan_data = False

    def move(self):
        twist = Twist()
        dist = self.front_distance
        if dist < self.min_safe_distance:
            twist.linear.x = 0.0
            twist.angular.z = random.uniform(-self.max_angular_speed, self.max_angular_speed)
            rospy.loginfo_throttle(1, f"Obstacle! dist={dist:.2f} → turning")
        else:
            twist.linear.x = min(self.max_linear_speed, dist / 2.0)
            twist.angular.z = random.uniform(-1, 1) * (self.max_angular_speed / (dist + 0.1))
            rospy.loginfo_throttle(1, f"Wandering: dist={dist:.2f}, lin={twist.linear.x:.2f}, ang={twist.angular.z:.2f}")
        self.cmd_pub.publish(twist)

if __name__ == "__main__":
    try:
        WanderBotSafety()
    except rospy.ROSInterruptException:
        pass

