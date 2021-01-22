
#!/usr/bin/env python
import numpy as np
import rospy
from std_msgs.msg import Int8, String
from sensor_msgs.msg import Image, CameraInfo


class RGBD2XYZ():
    def __init__(self):
        self.P = []
        self.w = 640
        self.h = 480
        self.fx = 0
        self.fy = 0
        self.cx = 0
        self.cy = 0
        self.tx = 0
        self.ty = 0
        self.depthScale = 1
        ################################################################
        self.imu_sub = rospy.Subscriber('/airsim_node/PhysXCar/front_left_bumblebee/DepthPlanner/camera_info', CameraInfo, self.callback_P)

    def callback_P(self, data):
        self.P = data.P
        rospy.loginfo(self.P)

    def defineP(self, arr):  # pass a list
        self.P = arr
        self.fx = self.P[0]
        self.fy = self.P[5]
        self.cx = self.P[2]
        self.cy = self.P[6]
        self.tx = self.P[3]
        self.ty = self.P[7]
        return self.P

    def calcZ(self, d):
        z = d / self.depthScale
        return z

    def calcX(self, u, z):
        X = (u - self.cx)*z/self.fx
        return X

    def calcY(self, v, z):
        Y = (v - self.cy)*z/self.fy
        return Y

    def xyz_array(self, dep_array):
        array_xyz = []
        for i in range(0, self.h):
            for j in range(0, self.w):
                z = self.calcZ(dep_array[i+j])
                y = self.calcY(i, z)
                x = self.calcX(j, z)
                arr = [x, y, z]
                array_xyz.append(arr)
        return array_xyz


if __name__ == "__main__":

    try:
        #       fx          cx             fy      cy
        arr = [320.0, 0.0, 320.0, 0.0, 0.0, 320.0, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0]

        depth_arr = np.genfromtxt("dataset/data/left/depth_1.txt", delimiter=',')
        depth_arr.dtype
        print(depth_arr)

        obj = RGBD2XYZ()
        obj.defineP(arr)
        xyz = obj.xyz_array(depth_arr)

        print(xyz)  # takes long time to execute
        print("Done!")

    except rospy.ROSInterruptException:
        print("Failed!")
        pass