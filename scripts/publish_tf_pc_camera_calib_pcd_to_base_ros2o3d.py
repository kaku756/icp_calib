#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
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
import numpy as np
from ros_numpy import numpify
from ros_numpy.point_cloud2 import split_rgb_field

from lib_cloud_conversion_between_Open3D_and_ROS import convertCloudFromOpen3dToRos, convertCloudFromRosToOpen3d
import sys
sys.path.append("/home/finibo/ws1/catkin_ws/src/finibo_camera_callib/scripts")
from lib_conversion_between_transform_and_quaternion import   convertRostfTotransform


def callback(msg1,msg2):
    # print msg1.data
    try:

        fname=str(time.time())

        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer)
        transform1 = tf_buffer.lookup_transform("workspace_link", "camera3_depth_optical_frame", rospy.Time())      #3B
        transform2 = tf_buffer.lookup_transform("workspace_link", "camera4_depth_optical_frame", rospy.Time())         #4B
        transform12 = tf_buffer.lookup_transform("camera4_depth_optical_frame", "camera3_depth_optical_frame", rospy.Time())  # 4B

        pcd1_transformed = do_transform_cloud(msg1, transform1)                     #convert pcd from camera 3,4 to work_link
        pcd2_transformed = do_transform_cloud(msg2, transform2)

        pcd1_o3d=convertCloudFromRosToOpen3d(pcd1_transformed)                      #conver pcd from Pointcloud2 to open3d Pointcloud
        pcd2_o3d=convertCloudFromRosToOpen3d(pcd2_transformed)

        center1 = vol.crop_point_cloud(pcd1_o3d)                                    #crop
        center2 = vol.crop_point_cloud(pcd2_o3d)

        p1, q1 = transform_to_pq(transform1)                                        #use homogeneous Matrix to conver back to camera coordination
        # print("trans2",p1,q1)
        T3b=convertRostfTotransform(p1,q1)
        print("T3b")#  T3B
        print(T3b)
        Tb3=np.linalg.inv(T3b)
        print("inv calc Tb3")
        print(Tb3)

        transform02 = tf_buffer.lookup_transform( "camera3_depth_optical_frame","workspace_link", rospy.Time())  # 4B
        p1, q1 = transform_to_pq(transform02)
        Tb30=convertRostfTotransform(p1,q1)
        print("tf Tb3")
        print(Tb30)

        p2, q2 = transform_to_pq(transform2)                                        #  T4b
        T4b=convertRostfTotransform(p2,q2)
        Tb4=np.linalg.inv(T4b)

        p12, q12 = transform_to_pq(transform12)                                        #  T4b
        T34=convertRostfTotransform(p12,q12)

        source = center1.transform(Tb3)                                             #convert pcd back to camera frame
        target = center2.transform(Tb4)


        trans_init=T34                                                              #init Matrix
        evaluation = o3d.registration.evaluate_registration(source, target,         #init result
                                                            threshold, trans_init)
        # print("Trans 34",transform12.transform)
        print("tf T34")
        transform340 = tf_buffer.lookup_transform("camera4_depth_optical_frame", "camera3_depth_optical_frame", rospy.Time())  # 4B
        p120, q120 = transform_to_pq(transform340)                                        #  T4b
        T340=convertRostfTotransform(p120,q120)
        print(T340)

        print("calc b4")
        print(np.dot(Tb3,T34))

        print("tf b4")
        print(Tb4)


        print("T3b inv Tb3")
        print(Tb3)
        print(T34)
        print("Init evaluation :",evaluation)
        print("Init workspace_link to camera4_depth_optical_frame is :")
        print("tf tb4")
        print("init calc tb4")
        print(np.dot(Tb3,T34))
        print("init tf Tb4")
        print(Tb4)

        print("Apply point-to-point ICP")
        reg_p2p = o3d.registration.registration_icp(                                #P2P ICP result
            source, target, threshold, trans_init,
            o3d.registration.TransformationEstimationPointToPoint())
        print("ICP T34")
        print(reg_p2p)
        print("point-to-point ICP Transformation is:")
        print(reg_p2p.transformation)
        print("result Tb4")
        print(np.dot(Tb3,np.array(reg_p2p.transformation)))







#-----------------------------------
        # try:
        #     (trans1, rot1) = tf_listener.lookupTransform('camera3_link', 'workspace_link', rospy.Time(0))
        # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        #     print("err")
        #



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



        print "------------written in yaml------------"

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




if __name__ == "__main__":

    directory="./"





    vol = o3d.visualization.read_selection_polygon_volume("/home/finibo/ws1/libs/test/Open3D/examples/TestData/Crop/finibo_cropped.json")
    threshold = 0.02
    trans_init = np.asarray([[1, 0.0, -0.0, 0.],                  #変換無し
                             [-0.0, 1, -0.0, 0.0],
                             [0.0, 0.0, 1.0, 0],
                             [0.0, 0.0, 0.0, 1.0]])


    listener()