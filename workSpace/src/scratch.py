#!/usr/bin/env python
import numpy as np
import rospy
from std_msgs.msg import Int8, String
from sensor_msgs.msg import Image, CameraInfo, PointCloud, ChannelFloat32
from geometry_msgs.msg import Point32, Point
from cv_bridge import CvBridge
import tf

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
        ################################################################
        # contains the RGB values of the image
        self.rgb = np.zeros((self.h, self.w, 3), dtype=int)
        # self.listener = tf.TransformListener()


    def getRGB(self, Image):
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(
            Image, desired_encoding='rgb8')  # the images comes in BGR8
        self.rgb = cv_image  # the image after converting it into openCV format -ndarray-

    def calcPCD(self, depthImage):  # returns a PointCloud object
        self.ID += 1
        print(self.ID)
        #############################
        # getting the RGB image from the scene topic
        # imageScene = rospy.Subscriber(
            # '/airsim_node/PhysXCar/front_left_bumblebee/Scene', Image, self.getRGB, queue_size=1)  # queue size of length 1 i.e. hold only one object and process it
        # decalring a point cloud variable
        pcd = PointCloud()
        # initializing the frame id, this is a mechanical frame
        pcd.header.frame_id = "front_left_bumblebee_body"
        # initializing the frame time stamp
        # Note you need to call rospy.init_node() before it; to work properly
        pcd.header.stamp = rospy.Time.now()
        # intializing the points list
        pcd.points = []
        # intializing the channels list
        pcd.channels = []
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
                # always declare a new object to avoid errors due to bad references
                pcd.points.append(Point32(-x[i][j], y[i][j], z[i][j]))
                # pcd.channels.append(ChannelFloat32(
                #     "rgb", [self.rgb[i][j][0], self.rgb[i][j][1], self.rgb[i][j][2]]))

        # self.listener.waitForTransform(
        #     "/world_ned", "/PhysXCar/front_left_bumblebee_body", rospy.Time.now(), rospy.Duration(4.0))
        # pcd_out = self.listener.transformPointCloud("/world_ned", pcd)
        return pcd


def callback_calcPCD(data):
    global obj
    XYZ_pub = rospy.Publisher(
        "PointCloud_ANM", PointCloud, queue_size=1)  # queue size of length 1 i.e. hold only one object and process it
    # the depth image is in a float format, we combine some consecutive values to create a single accurate one
    distances = np.fromstring(data.data, dtype=np.float32)
    depthImage = np.reshape(distances, (data.height, data.width))
    # publish the point cloud
    XYZ_pub.publish(obj.calcPCD(depthImage))


if __name__ == "__main__":

    try:
        rospy.init_node("ANM", anonymous=True)
        obj = RGBD2XYZ()
        depth_sub = rospy.Subscriber(
            '/airsim_node/PhysXCar/front_left_bumblebee/DepthPlanner', Image, callback_calcPCD, queue_size=1)  # queue size of length 1 i.e. hold only one object and process it
        rospy.spin()

    except rospy.ROSInterruptException:
        print("Failed!")
        pass
