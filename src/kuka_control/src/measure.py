#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import struct
import socket
import numpy as np
from tqdm import tqdm
import json
import os

# Network settings
IP_ADDR = '192.168.1.100'
PORT = 2001

# Command types and parameters
CMD_TYPE_SENSOR_TRANSMIT = '07'
SENSOR_TRANSMIT_TYPE_START = '01'
SENSOR_TRANSMIT_TYPE_STOP = '00'
CMD_TYPE_SET_CURRENT_TARE = '15'
SET_CURRENT_TARE_TYPE_NEGATIVE = '01'

s = None

def connect_socket():
    global s
    if s is not None:
        s.close()
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(5.0)
    s.connect((IP_ADDR, PORT))

def recvMsg():
    recvData = bytearray(s.recv(2))
    while len(recvData) < recvData[0]:
        recvData += bytearray(s.recv(recvData[0] - len(recvData)))
    return recvData

def calibrate():
    rospy.loginfo("Calibration starting in .....")
    rospy.sleep(3)
    
    rospy.loginfo("Calibrating sensor, please ensure no force is applied...")
    initial_readings = []
    for _ in tqdm(range(2000), desc="Calibrating", unit="readings"):
        recvData = recvMsg()
        force_vector = [
            struct.unpack('!d', recvData[2:10])[0],
            struct.unpack('!d', recvData[10:18])[0],
            struct.unpack('!d', recvData[18:26])[0]
        ]
        initial_readings.append(force_vector)
    rospy.loginfo("Calibration complete!")
    
    initial_offsets = np.mean(initial_readings, axis=0)
    rospy.loginfo(f"Calibration offsets: {initial_offsets}")
    return initial_offsets


def collect_measurements(position_id, num_readings=1000):
    rospy.loginfo(f"Collecting {num_readings} measurements for position {position_id}")
    readings = []
    for _ in tqdm(range(num_readings), desc="Measuring", unit="readings"):
        recvData = recvMsg()
        force_vector = [
            struct.unpack('!d', recvData[2:10])[0],
            struct.unpack('!d', recvData[10:18])[0],
            struct.unpack('!d', recvData[18:26])[0]
        ]
        readings.append(force_vector)
    
    new_measurement = {
        "position_id": position_id,
        "readings": readings
    }

    # Read existing data
    if os.path.exists('measurements.json'):
        with open('measurements.json', 'r') as f:
            existing_data = json.load(f)
    else:
        existing_data = []

    # Append new measurement
    existing_data.append(new_measurement)

    # Write updated data back to the file
    with open('measurements.json', 'w') as f:
        json.dump(existing_data, f, indent=4)

    rospy.loginfo(f"Measurements for position {position_id} stored successfully")

def measure_callback(data):
    rospy.loginfo(f"measure callback is being called!!!!!!!!!!!!!")
    global s
    if data.data == "calibrate":
        try:
            connect_socket()

            sendData = '03' + CMD_TYPE_SET_CURRENT_TARE + SET_CURRENT_TARE_TYPE_NEGATIVE
            sendData = bytearray.fromhex(sendData)
            s.send(sendData)
            recvData = recvMsg()

            sendData = '03' + CMD_TYPE_SENSOR_TRANSMIT + SENSOR_TRANSMIT_TYPE_START
            sendData = bytearray.fromhex(sendData)
            s.send(sendData)
            recvData = recvMsg()

            offsets = calibrate()

            measure_pub = rospy.Publisher('change_position_topic', String, queue_size=10)
            measure_pub.publish("calibration_done")
        except (socket.timeout, OSError) as e:
            rospy.logerr(f"Socket error during calibration: {e}")

    else:

        try:

            position_id = data.data
            collect_measurements(int(position_id))
            
            change_position_pub = rospy.Publisher('change_position_topic', String, queue_size=10)
            next_position_id = int(position_id) + 1
            change_position_pub.publish(str(next_position_id))
        except (socket.timeout, OSError) as e:
            rospy.logerr(f"Socket error during measurement collection: {e}")

def measure_listener():
    rospy.init_node('measure', anonymous=True)
    rospy.Subscriber('measure_topic', String, measure_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        measure_listener()
    except rospy.ROSInterruptException:
        pass
