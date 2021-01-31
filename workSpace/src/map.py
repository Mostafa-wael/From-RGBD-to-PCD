#!/usr/bin/env python
import numpy as np
import rospy
from sensor_msgs.msg import Image, PointCloud
from nav_msgs.msg import OccupancyGrid
import tf


class createMap():
    def __init__(self):
        self.ID = 0
        self.listener = tf.TransformListener()
        self.PCD = PointCloud()
        # occupancyGrid is a row major 1D list, loc = width * Y(Row) + X(Col)
        self.mapWidth = 500
        self.mapHeight = 500
        self.RES = 1
        ###
        self.grid_pub = rospy.Publisher(
        "OccupancyGrid_ANM", OccupancyGrid, queue_size=1)  # queue size of length 1 i.e. hold only one object and process it
        self.grid = OccupancyGrid()
        self.grid.info.height = self.mapWidth
        self.grid.info.width = self.mapHeight
        self.grid.info.origin.position.x = -self.mapWidth / (2 * self.RES)
        self.grid.info.origin.position.y = -self.mapHeight / (2 * self.RES)
        self.grid.info.origin.position.z = 0
        self.grid.info.origin.orientation.x = 0
        self.grid.info.origin.orientation.y = 0
        self.grid.info.origin.orientation.z = 0
        self.grid.info.origin.orientation.w = 1
        self.grid.info.resolution = 1.0 / self.RES
        self.grid.header.frame_id = "world_ned"

    def printPCDInFile(self, fileName, pcd):
        f = open(str(fileName) + ".txt", "w")
        for i in range(0, 480):
            for j in range(0, 640):
                f.write(str("X: "+str(pcd.points[i+j].x) + ", Y: " + str(
                        pcd.points[i+j].y) + ", Z: " + str(pcd.points[i+j].z) + "\n"))
        f.close()
        print("File Is Done")

    def filter(self, pcd):
        self.ID += 1
        print(self.ID)
        ###
        pcdFilter = [0]*self.mapWidth*self.mapHeight
        for point in pcd.points:
            x = int(point.x)
            y = int(point.y)
            if point.z < 10 and point.z > 2 and x < self.mapWidth and y < self.mapHeight:
               pcdFilter[self.mapWidth * y + x] = 127
                
        # for i in range(0, 480):
        #     for j in range(0, 640):
        #             if pcd.points[i+j].z < 10 and pcd.points[i+j].z > 2:
        #                pcdFilter[self.mapWidth * i + j] = 127
        self.grid.data = pcdFilter
        ###
        self.grid.header.stamp = rospy.Time.now()
        self.grid.info.map_load_time = rospy.Time.now()
        ###
        self.grid_pub.publish(self.grid)
        

    def callback_trasnformPCD(self, pcd_in):
        try:
            self.listener.waitForTransform(
                "world_ned", "front_left_bumblebee_body", pcd_in.header.stamp, rospy.Duration(1.0))
            self.PCD = self.listener.transformPointCloud("world_ned", pcd_in)
            ###
            # pcdFilter_pub = rospy.Publisher(
            # "PointCloudTrans_ANM", PointCloud, queue_size=1)  # queue size of length 1 i.e. hold only one object and process it
            # pcdFilter_pub.publish(self.PCD)
            ###
            # if self.ID < 3:
            #     self.printPCDInFile("test1",pcd_in)
            #     self.printPCDInFile("test2",self.PCD)
            self.filter(self.PCD)
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
