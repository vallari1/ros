#!/usr/bin/env python3
import rospy, actionlib, time
from robot_actions.msg import PickPlaceAction, PickPlaceGoal

rospy.init_node('pick_place_client')
client = actionlib.SimpleActionClient('pick_place_action', PickPlaceAction)
client.wait_for_server()
goal = PickPlaceGoal(x_pick=1, y_pick=2, z_pick=0.5, x_place=3, y_place=4, z_place=0.5)
client.send_goal(goal)
time.sleep(3)
client.cancel_goal()
client.wait_for_result()
print(client.get_result())
