#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import message_filters
from std_msgs.msg import String
import sensor_msgs.point_cloud2 as pc2
import tf2_ros
import tf2_py as tf2
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
import time
import pickle


import numpy as np
from ros_numpy import numpify
from ros_numpy.point_cloud2 import split_rgb_field














def callback(msg1,msg2):
    # print msg1.data
    try:

        fname=str(time.time())

        # msg1_transformed=tf.TransformerROS.transformPointCloud("workspace_link",msg1)
        # msg2_transformed=tf.TransformerROS.transformPointCloud("workspace_link",msg2)

        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer)
        transform1 = tf_buffer.lookup_transform("workspace_link", "camera3_depth_optical_frame", rospy.Time())
        transform2 = tf_buffer.lookup_transform("workspace_link", "camera4_depth_optical_frame", rospy.Time())

        pcd1_transformed = do_transform_cloud(msg1, transform1)
        pcd2_transformed = do_transform_cloud(msg2, transform2)



        # cloud_tmp_1=numpify(msg1_transformed)
        cloud_tmp_1=numpify(pcd1_transformed)

        cloud_split_1 = split_rgb_field(cloud_tmp_1)
        np1=np.stack(cloud_split_1[f] for f in ('x', 'y', 'z',"r","g","b")).T

        # cloud_tmp_2=numpify(msg2_transformed)
        cloud_tmp_2=numpify(pcd2_transformed)

        cloud_split_2 = split_rgb_field(cloud_tmp_2)
        np2=np.stack(cloud_split_2[f] for f in ('x', 'y', 'z',"r","g","b")).T



        np.savez("/home/finibo/ws1/data/"+fname+"_trans2",np1,np2)



        print "------------"

    except :pass
    # print msg2.data

def listener():
    rospy.init_node("icp_registration",anonymous=True)

    # tf_listener = tf.TransformListener()

    sub1=message_filters.Subscriber("/picker_robot/camera3/depth/color/points",PointCloud2)
    sub2=message_filters.Subscriber("/picker_robot/camera4/depth/color/points",PointCloud2)
    #
    # sub1=message_filters.Subscriber("/picker_robot/camera4/depth/camera_info",CameraInfo)
    # sub2=message_filters.Subscriber("/picker_robot/camera3/depth/camera_info",CameraInfo)

    # if not rospy.is_shutdown():
    #     try:
    #         (trans, rot) = tf_listener.lookupTransform('camera4_link', 'camera3_link', rospy.Time(0))
    #     except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
    #         print("err")



    queue_size = 10
    fps = .1
    delay = 1 / fps * 0.5


    mf=message_filters.ApproximateTimeSynchronizer([sub1,sub2],queue_size,delay)
    mf.registerCallback(callback)


    rospy.spin()

if __name__ == "__main__":






    listener()