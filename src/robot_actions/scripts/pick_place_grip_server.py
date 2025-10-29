#!/usr/bin/env python3
import rospy, actionlib, time
from robot_actions.msg import PickPlaceGripAction, PickPlaceGripFeedback, PickPlaceGripResult

def execute(goal):
    feedback, result = PickPlaceGripFeedback(), PickPlaceGripResult()
    grip_threshold = 5.0
    rate = rospy.Rate(2)
    for phase, (tx, ty, tz) in [('Moving to pick', (goal.x_pick, goal.y_pick, goal.z_pick)),
                                ('Grasping', (goal.x_pick, goal.y_pick, goal.z_pick)),
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
            rate.sleep()
        if phase == 'Grasping':
            time.sleep(1)
            result.sufficient_grip = goal.grip_strength >= grip_threshold
    result.result_message = "Pick and place completed with grip strength %.2f" % goal.grip_strength
    server.set_succeeded(result)

if __name__ == '__main__':
    rospy.init_node('pick_place_grip_server')
    server = actionlib.SimpleActionServer('pick_place_action', PickPlaceGripAction, execute, False)
    server.start()
    rospy.spin()
