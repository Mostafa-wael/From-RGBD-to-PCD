#!/usr/bin/env python
import numpy as np
import rospy
from std_msgs.msg import Int8, String
from sensor_msgs.msg import Image, CameraInfo, PointCloud, ChannelFloat32
from geometry_msgs.msg import Point32, Point
from cv_bridge import CvBridge
import tf
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud


class createMap():
    def __init__(self):
        self.ID = 0
        self.listener = tf.TransformListener()
        self.PCD = PointCloud()

    def filterZ(self, threshold):
        pcdArray = np.asarray(self.PCD.points)
        # filter array for values of z close to ground level
        pcdArray = pcdArray[pcdArray.z >= -18 and pcdArray.z <= 18]

    def callback_trasnformPCD(self, pcd_in):
        try:
            self.ID += 1
            print(self.ID)
            ###
            self.listener.waitForTransform(
                "/world_ned", "/front_left_bumblebee_body", pcd_in.header.stamp, rospy.Duration(1.0))
            self.PCD = self.listener.transformPointCloud("world_ned", pcd_in)
            ###
            self.filterZ(10)
        except (tf.LookupException, tf.ConnectivityException):
            print("Error")
            pass


if __name__ == "__main__":

    try:
        rospy.init_node("ANM", anonymous=True)
        obj = createMap()
        PCD_sub = rospy.Subscriber(
            '/PointCloud_ANM', PointCloud, obj.callback_trasnformPCD, queue_size=1)  # queue size of length 1 i.e. hold only one object and process it
        ####
        rospy.spin()

    except rospy.ROSInterruptException:
        print("Failed!")
        pass
