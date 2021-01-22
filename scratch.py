
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
        self.defineParameters()

    def defineParameters(self):  # waits for a message
        data = rospy.wait_for_message( '/airsim_node/PhysXCar/front_left_bumblebee/DepthPlanner/camera_info', CameraInfo, 100)
        self.w = data.width
        self.h = data.height
        self.P = data.P
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
        rospy.init_node("camera_info")
        depth_arr = np.genfromtxt(
            "dataset/data/left/depth_1.txt", delimiter=',')
        depth_arr.dtype
        print(depth_arr)

        obj = RGBD2XYZ()
        xyz = obj.xyz_array(depth_arr)

        print(xyz)  # takes long time to execute
        print("Done!")

    except rospy.ROSInterruptException:
        print("Failed!")
        pass
