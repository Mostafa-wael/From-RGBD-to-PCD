#!/usr/bin/env python
import numpy as np
import rospy
from std_msgs.msg import Int8, String
from sensor_msgs.msg import Image, CameraInfo, PointCloud, ChannelFloat32
from geometry_msgs.msg import Point32, Point
from cv_bridge import CvBridge
import tf
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud


def callback_printPCD(PCD):
    print(PCD.header)

if __name__ == "__main__":

    try:
        rospy.init_node("ANM", anonymous=True)
        depth_sub = rospy.Subscriber(
            '/PointCloud_ANM', PointCloud, callback_printPCD, queue_size=1)  # queue size of length 1 i.e. hold only one object and process it
        rospy.spin()

    except rospy.ROSInterruptException:
        print("Failed!")
        pass