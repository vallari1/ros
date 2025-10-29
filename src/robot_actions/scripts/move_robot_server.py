#!/usr/bin/env python3
import rospy, actionlib
from robot_actions.msg import MoveRobotAction, MoveRobotFeedback, MoveRobotResult
import math, time

def move_robot_server():
    server = actionlib.SimpleActionServer('move_robot', MoveRobotAction, execute, False)
    server.start()
    rospy.spin()

def execute(goal):
    feedback = MoveRobotFeedback()
    result = MoveRobotResult()
    rate = rospy.Rate(1)
    x, y = 0.0, 0.0
    while not rospy.is_shutdown():
        if abs(x - goal.x) < 0.01 and abs(y - goal.y) < 0.01:
            result.result_message = "Reached target (%.2f, %.2f)" % (goal.x, goal.y)
            server.set_succeeded(result)
            break
        x += (goal.x - x) * 0.1
        y += (goal.y - y) * 0.1
        feedback.current_x = x
        feedback.current_y = y
        server.publish_feedback(feedback)
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('move_robot_server')
    server = actionlib.SimpleActionServer('move_robot', MoveRobotAction, execute, False)
    server.start()
    rospy.spin()
