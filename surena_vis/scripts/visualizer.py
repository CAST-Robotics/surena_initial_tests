#!/usr/bin/env python3
import matplotlib.pyplot as plt
from matplotlib.dates import DateFormatter
from matplotlib.patches import Rectangle
import numpy as np
import random

import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point
from collections import deque

class RobotVisualizer:
    def __init__(self, dt):
        self.dt_ = dt
        self.footWidth_ = 0.1
        self.footHeight_ = 0.2

        rospy.init_node('robot_visualizer', anonymous=True)
        self.rate_ = rospy.Rate(int(1 / self.dt_))
        rospy.Subscriber("/surena/inc_joint_state", JointState, self.incJointCallback)
        rospy.Subscriber("/surena/foot_steps", Point, self.footStepCallback)

        self.incJointData_ = deque(maxlen=100)
        self.footStepData_ = []
        
        self.fig_, self.ax_ = plt.subplots()
        plt.ion()
        plt.show()

    def incJointCallback(self, data):
        self.incJointData_.append((data.header.stamp.to_sec(), data.position[0] + random.random()))
        # print(data.header.stamp.to_sec())

    def footStepCallback(self, data):
        self.footStepData_.append(data)
        print(data.point)

    def updatePlot(self):
        self.ax_.clear()
        # self.ax_.plot([t for t, inc in self.incJointData_], [inc for t, inc in self.incJointData_], label='inc_joint')

        for i in range(len(self.footStepData_)):
            x = self.footStepData_[i].x
            y = self.footStepData_[i].y
            rectangle = Rectangle((x - self.footHeight_/2, y - self.footWidth_/2), self.footHeight_, self.footWidth_, fill=False)
            self.ax_.add_patch(rectangle)

        self.ax_.legend()
        self.fig_.canvas.draw()
        self.fig_.canvas.flush_events()

    def spin(self):
        while not rospy.is_shutdown():
            self.updatePlot()
            self.rate_.sleep()

if __name__ == '__main__':
    vis = RobotVisualizer(0.005)
    vis.spin()