#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import json
import time

positions_file = '/root/catkin_ws/src/kuka_control/src/positions.json'

def change_position_callback(data):
    try:
        position_id = int(data.data)
    except ValueError:
        rospy.logwarn(f"Received non-integer position ID: {data.data}")
        return

    with open(positions_file, 'r') as f:
        positions = json.load(f)

    if position_id < len(positions):
        position = positions[position_id]
        rospy.loginfo(f"Moving to position {position_id}: {position}")
        # Simulate moving the robotic arm
        time.sleep(2)  # Simulate time taken to move

        measure_pub = rospy.Publisher('measure_topic', String, queue_size=10)
        measure_pub.publish(f"{position_id}")

        rospy.loginfo(f"made measurement")
        # Wait for measurements to complete before incrementing position ID
        # time.sleep(2)
        # next_position_id = position_id + 1
        # change_position_pub = rospy.Publisher('change_position_topic', String, queue_size=10)
        # change_position_pub.publish(str(next_position_id))
    else:
        rospy.loginfo("All positions have been measured.")

def change_position_listener():
    rospy.init_node('change_position', anonymous=True)
    rospy.Subscriber('change_position_topic', String, change_position_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        change_position_listener()
    except rospy.ROSInterruptException:
        pass
