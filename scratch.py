#!/usr/bin/env python
import numpy as np
import rospy
from std_msgs.msg import Int8, String
from sensor_msgs.msg import Image, CameraInfo, PointCloud
from geometry_msgs.msg import Point32

# Projection/camera matrix
#     [fx'  0  cx' Tx]
# P = [ 0  fy' cy' Ty]
#     [ 0   0   1   0]

class RGBD2XYZ():
    def __init__(self):
        self.P = []
        self.depthScale = 1
        self.ID = 0  # used for testing
        ################################################################
        data = rospy.wait_for_message(
            '/airsim_node/PhysXCar/front_left_bumblebee/DepthPlanner/camera_info', CameraInfo, 10)
        self.w = data.width
        self.h = data.height
        self.P = data.P
        self.fx = self.P[0]
        self.fy = self.P[5]
        self.cx = self.P[2]
        self.cy = self.P[6]
        self.tx = self.P[3]
        self.ty = self.P[7]

    def calcPCD(self, depthImage):  # returns a PointCloud object
        print(self.ID)
        self.ID += 1
        #############################
        # decalring a point cloud variable
        pcd = PointCloud()
        # initializing the frame id, this is a mechanical frame
        pcd.header.frame_id = "front_left_bumblebee_body"
        # intializing the points list
        pcd.points = []
        # initializing rows
        rows = np.zeros((self.h, self.w), dtype=float)
        rows[:, :] = np.arange(0, self.w)
        # initializing columns
        cols = np.zeros((self.h, self.w), dtype=float)
        cols[:, :] = np.array([np.arange(0, self.h)]).T
        # the calculations
        # for more about the equations, check this link: https://medium.com/yodayoda/from-depth-map-to-point-cloud-7473721d3f
        z = depthImage / self.depthScale
        y = (np.multiply((cols-self.cy), z))/self.fy
        x = (np.multiply((rows-self.cx), z))/self.fx
        for i in range(0, self.h):
            for j in range(0, self.w):
                tempPoint = Point32()  # always declare a new object to avoid errors due to bad references
                tempPoint.x, tempPoint.y, tempPoint.z = x[i][j], y[i][j], z[i][j]
                pcd.points.append(tempPoint)
        return pcd


def callback_calcPCD(data):
    global imageRec
    if imageRec == True:
        return
    ######
    global obj
    XYZ_pub = rospy.Publisher(
        "PointCloud_ANM", PointCloud, queue_size=1)  # queue size of length 1 i.e. hold only one object and process it
    # the depth image is in a float format, we combine some consecutive values to create a single accurate one
    distances = np.fromstring(data.data, dtype=np.float32)
    depthImage = np.reshape(distances, (data.height, data.width))
    # publish the point cloud
    XYZ_pub.publish(obj.calcPCD(depthImage))
    ######
    imageRec = False


if __name__ == "__main__":

    try:
        rospy.init_node("camera_info", anonymous=True)
        obj = RGBD2XYZ()
        imageRec = False
        depth_sub = rospy.Subscriber(
            '/airsim_node/PhysXCar/front_left_bumblebee/DepthPlanner', Image, callback_calcPCD, queue_size=1)  # queue size of length 1 i.e. hold only one object and process it
        rospy.spin()

    except rospy.ROSInterruptException:
        print("Failed!")
        pass
