#!/usr/bin/env python3
import rospy, actionlib
from robot_actions.msg import MoveRobotAction, MoveRobotGoal

rospy.init_node('move_robot_client')
client = actionlib.SimpleActionClient('move_robot', MoveRobotAction)
client.wait_for_server()
goal = MoveRobotGoal(x=5.0, y=5.0)
client.send_goal(goal)
client.wait_for_result()
print(client.get_result())
