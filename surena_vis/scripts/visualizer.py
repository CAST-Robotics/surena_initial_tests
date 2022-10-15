#!/usr/bin/env python3
import matplotlib.pyplot as plt
import numpy as np
import rospy
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist


class Visualize:
    def __init__(self, dt):
        plt.ion()
        self.dt = dt
        self.zmp = np.array([0, 0, 0])
        self.fig, self.ax = plt.subplots(3,1)
        print('done')

    def zmpCallback(self, data):
        self.zmp = np.vstack([self.zmp, np.array([data.x, data.y, data.z])])
        #print(self.zmp.shape)
        self.ax[0].plot(np.arange(self.zmp.shape[0]-1)*self.dt, self.zmp[1:, 0], color='blue')
        self.ax[0].plot(np.arange(self.zmp.shape[0]-1)*self.dt, self.zmp[1:, 1], color='red')
        self.ax[0].plot(np.arange(self.zmp.shape[0]-1)*self.dt, self.zmp[1:, 2], color='green')

        # self.ax[0].scatter((self.zmp.shape[0]-1)*self.dt, self.zmp[-1, 0], color='blue')
        # self.ax[0].scatter((self.zmp.shape[0]-1)*self.dt, self.zmp[-1, 1], color='green')
        # self.ax[0].scatter((self.zmp.shape[0]-1)*self.dt, self.zmp[-1, 2], color='red')

        self.spin()

    def comCallback(self, data):
        pass

    def xiCallback(self, data):
        pass

    def spin(self):
        rospy.init_node('visualizer', anonymous=True)
        self.zmpSub = rospy.Subscriber("/zmp_position", Point, self.zmpCallback)
        self.comSub = rospy.Subscriber("/com_data", Twist, self.comCallback)
        self.xiSub = rospy.Subscriber("/xi_data", Twist, self.xiCallback)
        plt.show(block=True)
    
    def close(self):
        plt.close()

if __name__ == '__main__':
    vis = Visualize(0.005)
    vis.spin()