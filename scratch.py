#!/usr/bin/env python
import numpy as np
import rospy
from std_msgs.msg import Int8, String
from sensor_msgs.msg import Image, CameraInfo, PointCloud
from geometry_msgs.msg import Point32


class RGBD2XYZ():
    def __init__(self):
        self.P = []
        self.depthScale = 1
        ################################################################
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
        ################################################################
        self.XYZ_pub = rospy.Publisher(
            "PointCloud_ANM", PointCloud, queue_size=1)  # queue size may change


    def xyz_array(self, depthImage):
        global ID
        print(ID)
        if ID == 0:
            f3 = open("depthimage.txt", "w")
            for row in depthImage:
                np.savetxt(f3, row)
            f3.close()
            print("depthImage: ")
            print(depthImage)
            print("depthImage Shape: ")
            print(depthImage.shape)
        pcd = PointCloud()
        pcd.header.frame_id = "front_left_bumblebee_body"
        pcd.points = []
        rows = np.zeros((480, 640), dtype=float)
        rows[:, :] = np.arange(0, 640)
        cols = np.zeros((480, 640), dtype=float)
        cols[:, :] = np.array([np.arange(0, 480)]).T
        z = depthImage / self.depthScale
        y = (np.multiply((cols-self.cy), z))/self.fy
        x = (np.multiply((rows-self.cx), z))/self.fx
        if ID == 0:
            ff = open("numpyresult.txt", "w")
        for i in range(0, self.h):
            for j in range(0, self.w):
                tempPointt = Point32()
                tempPointt.x, tempPointt.y, tempPointt.z = x[i][j], y[i][j], z[i][j]
                if ID == 0:
                    ff.write(str("X: "+str(tempPointt.x) + ", Y: " + str(
                        tempPointt.y) + ", Z: " + str(tempPointt.z) + "\n"))
                pcd.points.append(tempPointt)
        if ID == 0:
            ff.close()

        if ID == 0:
            print("z matrix : ")
            print(z)
            print("z matrix shape : ")
            print(z.shape)
            print("y matrix : ")
            print(y)
            print("y matrix shape : ")
            print(y.shape)
            print("x matrix : ")
            print(x)
            print("x matrix shape : ")
            print(x.shape)

        self.XYZ_pub.publish(pcd)
        ID += 1
        return pcd.points


def callback_depthImage(data):
    global imageRec
    if imageRec == True:
        return
    distances = np.fromstring(data.data, dtype=np.float32)
    depthImage = np.reshape(distances, (data.height, data.width))
    global obj
    XYZ_pub = obj.xyz_array(depthImage)
    imageRec = False


if __name__ == "__main__":

    try:
        rospy.init_node("camera_info")
        obj = RGBD2XYZ()
        ID = 0
        imageRec = False
        depth_sub = rospy.Subscriber(
            '/airsim_node/PhysXCar/front_left_bumblebee/DepthPlanner', Image, callback_depthImage, queue_size=1)
        rospy.spin()

    except rospy.ROSInterruptException:
        print("Failed!")
        pass
