#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import json

def polygon_publisher():
    pub = rospy.Publisher('/polygon_cmd', String, queue_size=10)
    rospy.init_node('polygon_publisher', anonymous=True)

    while not rospy.is_shutdown():
        print("\nChoose a shape:")
        print("1. Square\n2. Rectangle\n3. Triangle\n4. Circle\n5. Quit")
        choice = input("Enter your choice: ")

        if choice == '1':
            side = float(input("Enter side length: "))
            data = {"shape": "square", "side": side}
        elif choice == '2':
            length = float(input("Enter length: "))
            breadth = float(input("Enter breadth: "))
            data = {"shape": "rectangle", "length": length, "breadth": breadth}
        elif choice == '3':
            side = float(input("Enter side length: "))
            data = {"shape": "triangle", "side": side}
        elif choice == '4':
            radius = float(input("Enter radius: "))
            data = {"shape": "circle", "radius": radius}
        elif choice == '5':
            print("Exiting...")
            break
        else:
            print("Invalid choice!")
            continue

        rospy.loginfo(f"Publishing: {data}")
        pub.publish(json.dumps(data))
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        polygon_publisher()
    except rospy.ROSInterruptException:
        pass

