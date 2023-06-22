#!/usr/bin/env python3
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import json
import numpy as np
import random

import rospy
from rospkg import RosPack
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from collections import deque

class RobotVisualizer:
    def __init__(self, dt):

        self.dt_ = dt
        rospy.init_node('robot_visualizer', anonymous=True)
        self.rate_ = rospy.Rate(int(1 / self.dt_))
        rospy.Subscriber("/surena/inc_joint_state", JointState, self.incJointCallback)
        rospy.Subscriber("/surena/foot_steps", Point, self.footStepCallback)
        rospy.Subscriber("/surena/com_pose", PoseStamped, self.CoMCallback)

        self.incJointData_ = deque(maxlen=1000)
        self.CoMData_ = deque(maxlen=1000)
        self.footStepData_ = []

        self.plots = list()
        self.configPath_ = RosPack().get_path("surena_vis") + "/config/vis_config.json"
        self.parseConfig()

    def incJointCallback(self, data):
        self.incJointData_.append((data.header.stamp.to_sec(), data.position[0] + random.random()))

    def footStepCallback(self, data):
        if data.x == 0:
            self.footStepData_.clear()
        self.footStepData_.append(data)

    def CoMCallback(self, data):
        self.CoMData_.append((data.header.stamp.to_sec(), data.pose.position.x,
                                   data.pose.position.y, data.pose.position.z))

    def parseConfig(self):
        with open(self.configPath_) as f:
            config = json.load(f)

        for plot in config['plot']:
            plot_data = plot['Data']
            if plot_data == "com":
                self.plots.append((PosePlot("CoMPosePlot", "tz"), self.CoMData_))

    def updatePlots(self):
        for plot, data in self.plots:
            plot.update(data)

    def spin(self):
        while not rospy.is_shutdown():
            self.updatePlots()
            self.rate_.sleep()

class Plot:
    def __init__(self, title):
        self.fig_, self.ax_ = plt.subplots()
        self.fig_.canvas.set_window_title(title)

        plt.ion()
        plt.show()

    def update(self):
        pass

class PosePlot(Plot):
    def __init__(self, title, type):
        super().__init__(title)
        self.type_ = type

    def update(self, data):
        self.ax_.clear()
        t_data = [t for t, x, y, z in data]
        x_data = [x for t, x, y, z in data]
        y_data = [y for t, x, y, z in data]
        z_data = [z for t, x, y, z in data]

        if self.type_ == "xy":
            self.ax_.plot(x_data, y_data)
        elif self.type_ == "tx":
            self.ax_.plot(t_data, x_data)
        elif self.type_ == "ty":
            self.ax_.plot(t_data, y_data)
        elif self.type_ == "tz":
            self.ax_.plot(t_data, z_data)

        self.fig_.canvas.draw()
        self.fig_.canvas.flush_events()

class JointPlot(Plot):
    def __init__(self, title):
        super().__init__(title)

    def update(self, joint_data):
        self.ax_.clear()
        self.ax_.plot([t for t, inc in joint_data], [inc for t, inc in joint_data], label='inc_joint')
        self.fig_.canvas.draw()
        self.fig_.canvas.flush_events()

class FootStepPlot(Plot):
    def __init__(self, title):
        super().__init__(title)
        self.footWidth_ = 0.17
        self.footHeight_ = 0.25

    def update(self, foot_steps):
        self.ax_.clear()

        for i in range(len(foot_steps)):
            x = foot_steps[i].x
            y = foot_steps[i].y
            rectangle = Rectangle((x - self.footHeight_/2, y - self.footWidth_/2), self.footHeight_, self.footWidth_, fill=False)
            self.ax_.add_patch(rectangle)

        self.ax_.set_xlim(-0.2, 1.3)
        self.ax_.set_ylim(-0.2, 0.2)

        self.ax_.legend()
        self.fig_.canvas.draw()
        self.fig_.canvas.flush_events()