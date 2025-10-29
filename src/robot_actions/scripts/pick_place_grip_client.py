#!/usr/bin/env python3
import rospy, actionlib
from robot_actions.msg import PickPlaceGripAction, PickPlaceGripGoal

rospy.init_node('pick_place_grip_client')
client = actionlib.SimpleActionClient('pick_place_action', PickPlaceGripAction)
client.wait_for_server()
goal = PickPlaceGripGoal(x_pick=1, y_pick=2, z_pick=0.5, x_place=3, y_place=4, z_place=0.5, grip_strength=6.0)
client.send_goal(goal)
client.wait_for_result()
print(client.get_result())
