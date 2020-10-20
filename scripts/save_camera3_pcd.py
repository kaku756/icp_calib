#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import os
import message_filters
import open3d as o3d
from std_msgs.msg import String
import sensor_msgs.point_cloud2 as pc2
import tf2_ros
import tf
import tf2_py as tf2
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
import time
import pickle
from pyquaternion import Quaternion
from open3d.open3d.geometry import voxel_down_sample,estimate_normals
import yaml
import copy
import numpy as np
from ros_numpy import numpify
from ros_numpy.point_cloud2 import split_rgb_field

from lib_cloud_conversion_between_Open3D_and_ROS import convertCloudFromOpen3dToRos, convertCloudFromRosToOpen3d
import sys
sys.path.append("/home/finibo/ws1/catkin_ws/src/finibo_camera_callib/scripts")
from lib_conversion_between_transform_and_quaternion import   convertRostfTotransform


def callback(msg1,msg2):
    # print msg1.data
    # print "3"
    try:

        fname=str(time.time())

        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer)
        transform1 = tf_buffer.lookup_transform("hand_link", "camera3_depth_optical_frame", rospy.Time())      #3B
        transform2 = tf_buffer.lookup_transform("workspace_link", "camera4_depth_optical_frame", rospy.Time())         #4B
        transform12 = tf_buffer.lookup_transform("camera4_depth_optical_frame", "camera3_depth_optical_frame", rospy.Time())  # 4B

        pcd1_transformed = do_transform_cloud(msg1, transform1)                     #convert pcd from camera 3,4 to work_link
        pcd2_transformed = do_transform_cloud(msg2, transform2)

        pcd1_o3d=convertCloudFromRosToOpen3d(pcd1_transformed)                      #conver pcd from Pointcloud2 to open3d Pointcloud
        pcd2_o3d=convertCloudFromRosToOpen3d(pcd2_transformed)

        o3d.io.write_point_cloud("cam3_hand_link.ply",pcd1_o3d)


#-----------------------------------
        # try:
        #     (trans1, rot1) = tf_listener.lookupTransform('camera3_link', 'workspace_link', rospy.Time(0))
        # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        #     print("err")
        #

        #
        # pcd1_base = convertCloudFromOpen3dToRos(center1, frame_id="workspace_link")
        # pcd2_base = convertCloudFromOpen3dToRos(center2, frame_id="workspace_link")
        #
        # transform1_ = tf_buffer.lookup_transform("camera3_depth_optical_frame","workspace_link",  rospy.Time())
        # transform2_ = tf_buffer.lookup_transform("camera4_depth_optical_frame","workspace_link",  rospy.Time())
        #
        # pcd1_returned = do_transform_cloud(pcd1_base, transform1_)
        # pcd2_returned = do_transform_cloud(pcd2_base, transform2_)
        #
        # pcd1_returned_o3d=convertCloudFromRosToOpen3d(pcd1_returned)
        # pcd2_returned_o3d=convertCloudFromRosToOpen3d(pcd2_returned)
        #
        # o3d.io.write_point_cloud("/home/finibo/ws1/libs/test/Open3D/examples/TestData/pcd1_returned_ros2o3d.ply", pcd1_returned_o3d)
        # o3d.io.write_point_cloud("/home/finibo/ws1/libs/test/Open3D/examples/TestData/pcd2_returned_ros2o3d.ply", pcd2_returned_o3d)

        # estimate_normals(cloud=center1, search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
        # estimate_normals(cloud=center2, search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
        #
        # print("Apply point-to-plane ICP")
        # reg_p2p = o3d.registration.registration_icp(
        #     pcd1_returned_o3d, pcd2_returned_o3d, threshold, trans_init,
        #     o3d.registration.TransformationEstimationPointToPoint())
        # print(reg_p2p)
        # print("Transformation is:")
        # print(reg_p2p.transformation)


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

    # print "1"

    queue_size = 10
    fps = .1
    delay = 1 / fps * 0.5


    mf=message_filters.ApproximateTimeSynchronizer([sub1,sub1],queue_size,delay)
    mf.registerCallback(callback)

    # print "2"
    rospy.spin()


def transform_to_pq(msg_raw):
    """Convert a C{geometry_msgs/Transform} into position/quaternion np arrays

    @param msg: ROS message to be converted
    @return:
      - p: position as a np.array
      - q: quaternion as a numpy array (order = [x,y,z,w])
    """
    msg=msg_raw.transform
    p = np.array([msg.translation.x, msg.translation.y, msg.translation.z])
    q = np.array([msg.rotation.x, msg.rotation.y,
                  msg.rotation.z, msg.rotation.w])
    return p, q

def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp])


if __name__ == "__main__":

    # default_directory=os.path.expanduser("~/ws1/catkin_ws/src/finibo_camera_callib/config")



    print "cam3 listen"
    listener()