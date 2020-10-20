#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import message_filters
from std_msgs.msg import String
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField, CameraInfo
import time
import pickle
import tf
import numpy as np
from ros_numpy import numpify
from ros_numpy.point_cloud2 import split_rgb_field




def callback(msg1,msg2):
    # print msg1.data
    try:

        fname=str(time.time())

        cloud_tmp_1=numpify(msg1)
        cloud_split_1 = split_rgb_field(cloud_tmp_1)
        np1=np.stack(cloud_split_1[f] for f in ('x', 'y', 'z',"r","g","b")).T

        cloud_tmp_2=numpify(msg2)
        cloud_split_2 = split_rgb_field(cloud_tmp_2)
        np2=np.stack(cloud_split_2[f] for f in ('x', 'y', 'z',"r","g","b")).T


        print("msg1 header :",msg1.header)
        print("msg2 header :",msg2.header)

        np.savez("/home/finibo/ws1/data/"+fname,np1,np2)



        print "------------"

    except :pass
    # print msg2.data

def listener():
    rospy.init_node("icp_registration",anonymous=True)


    sub1=message_filters.Subscriber("/picker_robot/camera3/depth/color/points",PointCloud2)
    sub2=message_filters.Subscriber("/picker_robot/camera4/depth/color/points",PointCloud2)
    #
    # sub1=message_filters.Subscriber("/picker_robot/camera4/depth/camera_info",CameraInfo)
    # sub2=message_filters.Subscriber("/picker_robot/camera3/depth/camera_info",CameraInfo)




    queue_size = 10
    fps = .1
    delay = 1 / fps * 0.5


    mf=message_filters.ApproximateTimeSynchronizer([sub1,sub2],queue_size,delay)
    mf.registerCallback(callback)


    rospy.spin()

if __name__ == "__main__":

    listener()