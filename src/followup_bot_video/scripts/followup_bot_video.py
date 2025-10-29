#!/usr/bin/env python3
import rospy
import cv2
import math
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

def followup_bot(video_path):
    rospy.init_node('followup_bot_video', anonymous=True)
    pub = rospy.Publisher('/camera/image_raw', Image, queue_size=10)
    bridge = CvBridge()
    cap = cv2.VideoCapture(video_path)
    rate = rospy.Rate(30)

    if not cap.isOpened():
        rospy.logerr(f"Cannot open video: {video_path}")
        return

    rospy.loginfo("Publishing and processing video frames... Press 'q' to quit.")

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            rospy.loginfo("End of video reached.")
            break

        img_msg = bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        pub.publish(img_msg)

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 50, 150)
        h, w = edges.shape

        cv2.line(frame, (w // 2, 0), (w // 2, h), (0, 0, 255), 2)
        overlay = cv2.addWeighted(frame, 0.8, cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR), 0.4, 0)

        cv2.imshow("Followup Bot - Road Detection", overlay)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rospy.loginfo("User quit.")
            break

        rate.sleep()

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    import sys
    if len(sys.argv) < 2:
        print("Usage: rosrun followup_bot_video followup_bot_video.py <path_to_video>")
        sys.exit(1)

    followup_bot(sys.argv[1])

