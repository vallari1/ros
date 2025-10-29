#!/usr/bin/env python3
import rospy, actionlib, time
from robot_actions.msg import PickPlaceAction, PickPlaceFeedback, PickPlaceResult

def execute(goal):
    feedback, result = PickPlaceFeedback(), PickPlaceResult()
    rate = rospy.Rate(2)
    for phase, (tx, ty, tz) in [('Moving to pick', (goal.x_pick, goal.y_pick, goal.z_pick)),
                                ('Picking', (goal.x_pick, goal.y_pick, goal.z_pick)),
                                ('Moving to place', (goal.x_place, goal.y_place, goal.z_place)),
                                ('Placing', (goal.x_place, goal.y_place, goal.z_place))]:
        if server.is_preempt_requested():
            result.result_message = "Preempted during %s" % phase
            server.set_preempted(result)
            return
        for i in range(5):
            feedback.phase = phase
            feedback.current_x, feedback.current_y, feedback.current_z = tx, ty, tz
            server.publish_feedback(feedback)
            time.sleep(0.5)
        if phase == 'Picking':
            time.sleep(1)
    result.result_message = "Pick and place completed"
    server.set_succeeded(result)

if __name__ == '__main__':
    rospy.init_node('pick_place_server')
    server = actionlib.SimpleActionServer('pick_place_action', PickPlaceAction, execute, False)
    server.start()
    rospy.spin()
