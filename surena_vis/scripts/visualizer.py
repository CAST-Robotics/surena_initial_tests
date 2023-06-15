#!/usr/bin/env python3
import matplotlib.pyplot as plt
import numpy as np
import random

import rospy
from sensor_msgs.msg import JointState
from collections import deque

class RobotVisualizer:
    def __init__(self, dt):
        rospy.init_node('robot_visualizer', anonymous=True)
        self.zmpSub_ = rospy.Subscriber("/surena/inc_joint_state", JointState, self.incJointCallback)
        self.dt_ = dt
        self.iter = 0
        self.rate_ = rospy.Rate(int(1 / self.dt_))
        self.fig_, self.ax_ = plt.subplots()
        self.incJointData_ = deque(maxlen=100)
        plt.ion()
        plt.show()

    def incJointCallback(self, data):
        self.incJointData_.append((data.header.stamp.to_sec(), data.position[0] + random.random()))
        print(data.header.stamp.to_sec())

    def updatePlot(self):
        self.ax_.clear()
        self.ax_.plot([t for t, inc in self.incJointData_], [inc for t, inc in self.incJointData_], label='inc_joint')

        self.ax_.legend()
        self.fig_.canvas.draw()
        self.fig_.canvas.flush_events()

    def spin(self):
        while not rospy.is_shutdown():
            self.updatePlot()
            self.rate_.sleep()
    
    def close(self):
        plt.close()

if __name__ == '__main__':
    vis = RobotVisualizer(0.005)
    vis.spin()