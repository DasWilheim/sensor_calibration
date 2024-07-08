#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

def test_publisher():
    rospy.init_node('test_measure_publisher', anonymous=True)
    measure_pub = rospy.Publisher('measure_topic', String, queue_size=10)
    
    # Wait a moment to ensure the subscriber is ready
    rospy.sleep(1)
    
    # Publish a test message to trigger calibration
    rospy.loginfo("Publishing test calibration message")
    measure_pub.publish("calibrate")
    
    # Wait a bit before publishing the next message
    rospy.sleep(5)
    
    # Publish a test message to start measurements with position ID 1
    rospy.loginfo("Publishing test start_measurements message with position_id 1")
    measure_pub.publish("start_measurements 1")


    rospy.sleep(5)
    
    # Publish a test message to start measurements with position ID 1
    rospy.loginfo("Publishing test start_measurements message with position_id 1")
    measure_pub.publish("start_measurements 2")

if __name__ == '__main__':
    try:
        test_publisher()
    except rospy.ROSInterruptException:
        pass
