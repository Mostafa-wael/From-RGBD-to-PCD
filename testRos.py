#!/usr/bin/env python

import math
from math import sin, cos, pi
import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3   # #Note, I used camera here but lidar will be used taken
from sensor_msgs.msg import Imu                                         # in consideration lidar doesn't publish odom (eng monsef).     
from std_msgs.msg import Int8,String
#++++++++++++++++++++++++++++++++++
class od():

    def _init_(self):
  	   
  	   self.odom_pub = rospy.Publisher("ODOM", Odometry, queue_size=50) # queue size may change 
  	   self.imu_sub = rospy.Subscriber('RandIMU', Imu, self.callback_imu)  
  	   self.encoder_sub = rospy.Subscriber("Ticks", String, self.callback_encoder)
  	   self.odom_broadcaster = tf.TransformBroadcaster()
  	   self.Odom = Odometry()
  	   self.imu_data = Imu()
  	   self.ticks=0
  	   self.rate = rospy.Rate(10) # 10hz
  	   self.num_ticks_per_one_rev = 6266
  	   self.Wheel_diameter = 40
  	   self.distance_per_rev = (self.Wheel_diameter * pi) / self.num_ticks_per_one_rev

    def callback_imu(self,data):
        
  	   self.imu_data = data.orientation.z
  	   rospy.loginfo(self.imu_data.orientation.z)

    def callback_encoder(self,dta):
  	   self.ticks = dta.data
  	   self.Odom_Generator()

    def Odom_Generator(self):
        
        theta=0
        current_time=rospy.Time.now()
        while not rospy.is_shutdown():

            delta_theta = (self.imu_data).orientation.z
    	    distance_travelled = (int(self.ticks) / self.num_ticks_per_one_rev) * self.distance_per_rev #for total distance calculation
    	    x_position = 0 + distance_travelled * cos(theta)
    	    y_position = 0 + distance_travelled * sin(theta)
            odom_quat = tf.transformations.quaternion_from_euler(0, 0, theta)
            self.odom_broadcaster.sendTransform((x_position, y_position, 0.), odom_quat, current_time, "base_link", "odom")
            self.Odom.header.frame_id = "odom"
            self.Odom.pose.pose = Pose(Point(x_position, y_position, 0.), Quaternion(*odom_quat))
            self.odom_pub.publish(self.Odom)
            self.rate.sleep()
#************************
if __name__ == '_main_':
    rospy.init_node('odometry_publisher',anonymous=True)
    odom_obj = od()
    try:
        odom_obj.Odom_Generator()
    except rospy.ROSInterruptException:
        pass