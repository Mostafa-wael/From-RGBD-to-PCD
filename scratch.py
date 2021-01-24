#!/usr/bin/env python
import numpy as np
import rospy
from std_msgs.msg import Int8, String
from sensor_msgs.msg import Image, CameraInfo, PointCloud
from geometry_msgs.msg import Point32


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
        self.XYZ_pub = rospy.Publisher(
            "PointCloud_ANM", PointCloud, queue_size=1)  # queue size may change

    def defineParameters(self):  # waits for a message
        data = rospy.wait_for_message(
            '/airsim_node/PhysXCar/front_left_bumblebee/DepthPlanner/camera_info', CameraInfo, 100)
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

    def xyz_array(self, depthImage):
        global ID
        print(ID)
        pcd = PointCloud()
        temp = Point32()
        temp.z = 0
        temp.y = 0
        temp.x = 0
        pcd.points = []
        if ID == 0:
            f = open("depth_1.txt", "w")
        for i in range(0, self.h):
            for j in range(0, self.w):
                tempPoint = Point32()
                tempPoint.z = self.calcZ(depthImage[i][j])
                tempPoint.y = self.calcY(i, tempPoint.z)
                tempPoint.x = self.calcX(j, tempPoint.z)
                pcd.points.append(tempPoint)
                if ID == 0:
                    f.write(str("X: "+str(pcd.points[i+j].x) + ", Y: " + str(
                        pcd.points[i+j].y) + ", Z: " + str(pcd.points[i+j].z) + "\n"))
        if ID == 0:
            f.close()
        ######################
        if ID == 0:
            f = open("depth_2.txt", "w")
            for i in range(0, self.h):
                for j in range(0, self.w):
                    f.write(str("X: "+str(pcd.points[i+j].x) + ", Y: " + str(
                        pcd.points[i+j].y) + ", Z: " + str(pcd.points[i+j].z) + "\n"))
            f.close()
        self.XYZ_pub.publish(pcd)

        ID += 1
        return pcd.points


def callback_depthImage(data):
    distances = np.fromstring(data.data, dtype=np.float32)
    depthImage = np.reshape(distances, (data.height, data.width))
    global obj
    XYZ_pub = obj.xyz_array(depthImage)


if __name__ == "__main__":

    try:
        rospy.init_node("camera_info")
        obj = RGBD2XYZ()
        ID = 0
        depth_sub = rospy.Subscriber(
            '/airsim_node/PhysXCar/front_left_bumblebee/DepthPlanner', Image, callback_depthImage, queue_size=1)
        rospy.spin()

    except rospy.ROSInterruptException:
        print("Failed!")
        pass
