#!/usr/bin/env python
# -*- coding: utf-8 -*-
# import open3d
import numpy as np


import rospy
# from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2

from ros_numpy import numpify
import time
from ros_numpy.point_cloud2 import split_rgb_field
# import ros_numpy

import sys
sys.path.append("/home/finibo/ws1/libs/open3d_ros_pointcloud_conversion")

from lib_cloud_conversion_between_Open3D_and_ROS import convertCloudFromOpen3dToRos as ros2o3d




def callback(data):

    fname = str(time.time())

    cloud_tmp = numpify(data)
    cloud_split=split_rgb_field(cloud_tmp)



    # np1=np.stack(cloud_split[f] for f in ('x', 'y', 'z',"r","g","b")).T
    np1=np.stack(cloud_split[f] for f in ('x', 'y', 'z')).T


    np.save("/home/finibo/ws1/data/" + fname+"_", np1)

    print ("hahahaha")


def listener():
    rospy.init_node("icp_registration",anonymous=True)
    rospy.Subscriber("/picker_robot/camera4/depth/color/points",PointCloud2,callback)


    rospy.spin()
if __name__ == "__main__":

    listener()