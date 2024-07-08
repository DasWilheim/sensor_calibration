#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def keyboard_listener():
    rospy.init_node('command_listener', anonymous=True)
    measure_pub = rospy.Publisher('measure_topic', String, queue_size=10)
    change_position_pub = rospy.Publisher('change_position_topic', String, queue_size=10)
    
    while not rospy.is_shutdown():
        command = input("Enter command: ")
        if command == "start":
            rospy.loginfo("Starting calibration...")
            measure_pub.publish("calibrate")
        elif command.startswith("measure"):
            _, position_id = command.split()
            rospy.loginfo("Starting measurements...")
            change_position_pub.publish(position_id)

if __name__ == '__main__':
    try:
        keyboard_listener()
    except rospy.ROSInterruptException:
        pass
