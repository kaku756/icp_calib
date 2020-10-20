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
    try:

        fname=str(time.time())
        print "a"

        transform_w3 = tf_buffer.lookup_transform("workspace_link", "camera3_depth_optical_frame", rospy.Time())      #3B
        transform_w4 = tf_buffer.lookup_transform("workspace_link", "camera4_depth_optical_frame", rospy.Time())         #4B
        transform_43 = tf_buffer.lookup_transform("camera4_depth_optical_frame", "camera3_depth_optical_frame", rospy.Time())  # 4B
        transform_o4 = tf_buffer.lookup_transform("camera4_depth_optical_frame", "camera4_link", rospy.Time())  # 4B
        transform_wo4 = tf_buffer.lookup_transform("workspace_link", "camera4_link", rospy.Time())         #4B




        print "trans_wo4",transform_wo4
        p,q = transform_to_pq(transform_wo4)
        Two4=convertRostfTotransform(p,q)
        print "init two4",Two4
        # trans=tf_buffer.lookup_transform("hand_link","base_link",rospy.Time())
        #
        # print trans

        pcd1_transformed = do_transform_cloud(msg1, transform_w3)

        # temp=convertCloudFromRosToOpen3d(msg1)

        q1,p1=transform_to_pq(transform_w3)
        Tw3=convertRostfTotransform(q1,p1)

        # temp2=copy.deepcopy(temp).transform(Tw3)
        # o3d.io.write_point_cloud("cam3Tw3.ply",temp2)
        # o3d.io.write_point_cloud("cam3raw.ply",temp)

        # temp=convertCloudFromRosToOpen3d(pcd1_transformed)
        # o3d.io.write_point_cloud("cam3trans1.ply",temp)

        #convert pcd from camera 3,4 to work_link
        pcd2_transformed = do_transform_cloud(msg2, transform_w4)




        pcd1_o3d=convertCloudFromRosToOpen3d(pcd1_transformed)                      #conver pcd from Pointcloud2 to open3d Pointcloud
        pcd2_o3d=convertCloudFromRosToOpen3d(pcd2_transformed)

        center1 = vol.crop_point_cloud(pcd1_o3d)                                    #crop
        center2 = vol.crop_point_cloud(pcd2_o3d)


        # #--------------------
        #
        trans_init = np.asarray([[1, 0.0, -0.0, 0.],  # 変換無し
                                 [-0.0, 1, -0.0, 0.0],
                                 [0.0, 0.0, 1.0, 0],
                                 [0.0, 0.0, 0.0, 1.0]])

        "center 1 2 単位I"
        # draw_registration_result(center1, center2, trans_init)
        #
        # #--------------------

        p1, q1 = transform_to_pq(transform_w3)                                        #use homogeneous Matrix to conver back to camera coordination
        # print("trans2",p1,q1)
        Tw3=convertRostfTotransform(p1,q1)                                          #  T3B
        T3w=np.linalg.inv(Tw3)


        p2, q2 = transform_to_pq(transform_w4)                                        #  T4b
        Tw4=convertRostfTotransform(p2,q2)
        T4w=np.linalg.inv(Tw4)

        p12, q12 = transform_to_pq(transform_43)                                        #  T4b
        T43=convertRostfTotransform(p12,q12)

        source = center1.transform(T3w)        #  cam3                                   #convert pcd back to camera frame
        target = center2.transform(T4w)        #  cam4

        o3d.io.write_point_cloud("source.ply",source)
        o3d.io.write_point_cloud("target.ply",target)


        # #--------------------
        #
        # trans_init = np.asarray([[1, 0.0, -0.0, 0.],  # 変換無し
        #                          [-0.0, 1, -0.0, 0.0],
        #                          [0.0, 0.0, 1.0, 0],
        #                          [0.0, 0.0, 0.0, 1.0]])
        #
        #
        # #--------------------

        trans_init=T43                                                              #init Matrix
        evaluation = o3d.registration.evaluate_registration(source, target,         #init result
                                                            threshold, trans_init)

        "back to 3,4, with initT=T43 should match"
        draw_registration_result(source, target, trans_init)

        # print("Trans 34",transform_43.transform)
        # print("tf T43")
        # print("T3b inv Tw3")
        # print(Tw3)
        print("Tf published T43")
        print(T43)
        print("Init evaluation :",evaluation)
        print("Init workspace_link to camera4_depth_optical_frame is :")
        # print("tf tb4")
        # print("init calc tb4")
        # print(np.dot(Tw3,T43))
        # print("init tf Tb4")
        print(T4w)

        print("Apply point-to-point ICP")
        reg_p2p = o3d.registration.registration_icp(                                #P2P ICP result
            source, target, threshold, trans_init,
            o3d.registration.TransformationEstimationPointToPoint())
        print("ICP T43")
        print(reg_p2p)
        print("point-to-point ICP Transformation is:")
        print(reg_p2p.transformation)
        resultTw4=np.dot(Tw3,np.linalg.inv(np.array(reg_p2p.transformation)))
        print("result Tw4",resultTw4)

        p,q = transform_to_pq(transform_o4)
        To4=convertRostfTotransform(p,q)
        print "To4",To4
        resultTwo4=np.dot(resultTw4,To4)
        print "result Two4",resultTwo4



        print(resultTwo4)

        "ICP caulced should match"
        draw_registration_result(source, target, reg_p2p.transformation)

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
        camera="camera4"
        Q=Quaternion(matrix=resultTwo4)
        print directory
        print "w"
        with open(directory + "/{}_transform.yaml".format(camera), 'w') as f:
            f.write("eye_on_hand: false\n")
            f.write("robot_base_frame: workspace_link\n")
            f.write("tracking_base_frame: {}_link\n".format(camera))
            f.write("transformation: {")
            f.write("x: {}, ".format(resultTwo4[0,3]))
            f.write("y: {}, ".format(resultTwo4[1,3]))
            f.write("z: {}, ".format(resultTwo4[2,3]))
            f.write("qx: {}, ".format(Q[1]))
            f.write("qy: {}, ".format(Q[2]))
            f.write("qz: {}, ".format(Q[3]))
            f.write("qw: {} }}\n".format(Q[0]))


        print "------------written in yaml------------"

    except :pass
    # print msg2.data

def listener():

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


    mf=message_filters.ApproximateTimeSynchronizer([sub1,sub2],queue_size,delay)
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
    rospy.init_node("icp_registration",anonymous=True)

    directory = rospy.get_param('~config_dir', os.path.expanduser("~/ws1/catkin_ws/src/finibo_camera_callib/ICP_config"))
    if not os.path.exists(directory):
        os.mkdir(directory)

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)



    vol = o3d.visualization.read_selection_polygon_volume("/home/finibo/ws1/libs/test/Open3D/examples/TestData/Crop/finibo_cropped.json")
    threshold = 0.02
    trans_init = np.asarray([[1, 0.0, -0.0, 0.],                  #変換無し
                             [-0.0, 1, -0.0, 0.0],
                             [0.0, 0.0, 1.0, 0],
                             [0.0, 0.0, 0.0, 1.0]])

    print "listen"
    listener()