#!/usr/bin/env python3
import serial
import time
import rospy
from std_msgs.msg import Int32MultiArray

class bumpSensor:
    def __init__(self, port):
        self.serPort_ = serial.Serial(port=port, baudrate=115200)
        self.serPort_.close()
        self.serPort_.open()
        self.bumpPub_ = rospy.Publisher('SerialBump', Int32MultiArray, queue_size=10)
        self.rate_ = rospy.Rate(250)

    def parse_data(self, data):
        bump_data = []
        for i in range(4):
            bump_data.append(int(data[2 * i + 3:2 * i + 5]))
        bump_data.extend([0, 0, 0, 0])
        return bump_data


    def publish_data(self):
        while not rospy.is_shutdown():
            if(self.serPort_.in_waiting > 0):
                self.sensorString_ = self.serPort_.readline().decode('unicode_escape').strip()
                self.parsedData_ = Int32MultiArray()
                if(len(self.sensorString_) == 14):
                    self.parsedData_.data = self.parse_data(self.sensorString_)
                else:
                    self.parsedData_.data = [0, 0, 0, 0, 0, 0, 0, 0]
                self.bumpPub_.publish(self.parsedData_)
                


if __name__ == '__main__':
    rospy.init_node('bumpSensor', anonymous=True)
    bump = bumpSensor('/dev/ttyACM0')
    bump.publish_data()