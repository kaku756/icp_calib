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



def callback(data):
    assert isinstance(data,PointCloud2)
    # gen = pc2.read_points(data,field_names=("x","y","z","rgb"),skip_nans=True)
    # print gen.Header
    # time.sleep(1)
    # print msg1.data
    # print "3"
    rate = rospy.Rate(10.0)

    # try:

    fname=str(time.time())


    # transform1 = tf_buffer.lookup_transform("workspace_link", "camera3_depth_optical_frame", rospy.Time())      #3B
    # print transform1
    # transform = tf_buffer.lookup_transform("hand_link", "camera3_depth_optical_frame", rospy.Time())      #3B

    transform = tf_buffer.lookup_transform( "camera3_depth_optical_frame", "hand_link",rospy.Time())  # 3B

    transform_wh = tf_buffer.lookup_transform("hand_link", "workspace_link", rospy.Time())  # 3B


    p,q=transform_to_pq(transform)
    T3h=convertRostfTotransform(p,q)
    # print transform
    print T3h

    pcd_transformed_3h = do_transform_cloud(data, transform)      #cam3 to hand crop this

    data_o3d=convertCloudFromRosToOpen3d(data)
    o3d.io.write_point_cloud("cam3raw.ply",data_o3d)            # can delete
    pcd_o3d_3h = convertCloudFromRosToOpen3d(pcd_transformed_3h)
    o3d.io.write_point_cloud("3h_test.ply",pcd_o3d_3h)          #delete
    cropped = vol.crop_point_cloud(pcd_o3d_3h)
    Th3=np.linalg.inv(T3h)
    cropped_h3=copy.deepcopy(cropped).transform(Th3)
    o3d.io.write_point_cloud("crop_back.ply",cropped_h3)

    # o3d.draw_geometries([cropped_h3])
    pwh,qwh=transform_to_pq(transform_wh)
    Twh=convertRostfTotransform(pwh,qwh)
    threshold = 0.02
    trans_init = Th3

    source=pcd_stl
    target=cropped_h3

    # estimate_normals(cloud=source,search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
    # estimate_normals(cloud=target,search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))

    draw_registration_result(source, target, trans_init)
    print("Initial alignment")
    print trans_init
    evaluation = o3d.registration.evaluate_registration(source, target,
                                                        threshold, trans_init)
    print(evaluation)

    print("Apply point-to-point ICP")
    reg_p2p = o3d.registration.registration_icp(
        source, target, threshold, trans_init,
        o3d.registration.TransformationEstimationPointToPoint())
    print(reg_p2p)
    print("Transformation is:")
    print(reg_p2p.transformation)

    Th3_p=reg_p2p.transformation
    Tw3=np.dot(Twh,Th3_p)

    transform = tf_buffer.lookup_transform("camera3_link", "camera3_depth_optical_frame", rospy.Time())  # 3B
    p,q = transform_to_pq(transform)
    Topt3link=convertRostfTotransform(p,q)
    Tw3l=np.dot(Tw3,Topt3link)




    print("")
    draw_registration_result(source, target, reg_p2p.transformation)
    resT=Tw3l
    camera = "camera3"
    Q = Quaternion(matrix=resT)
    print dir
    with open(dir + "/{}_transform.yaml".format(camera), 'w') as f:
        f.write("eye_on_hand: false\n")
        f.write("robot_base_frame: workspace_link\n")
        f.write("tracking_base_frame: {}_link\n".format(camera))
        f.write("transformation: {")
        f.write("x: {}, ".format(resT[0, 3]))
        f.write("y: {}, ".format(resT[1, 3]))
        f.write("z: {}, ".format(resT[2, 3]))
        f.write("qx: {}, ".format(Q[0]))
        f.write("qy: {}, ".format(Q[1]))
        f.write("qz: {}, ".format(Q[2]))
        f.write("qw: {} }}\n".format(Q[3]))

    print "------------written in yaml------------"




    print Tw3
    print Tw3l


    #
    # print("Apply point-to-plane ICP")
    # reg_p2l = o3d.registration.registration_icp(
    #     source, target, threshold, trans_init,
    #     o3d.registration.TransformationEstimationPointToPlane())
    # print(reg_p2l)
    # print("Transformation is:")
    # print(reg_p2l.transformation)

    # dict=t2q(reg_p2l.transformation)

    print("")
    # draw_registration_result(source, target, reg_p2l.transformation)

    # o3d.write_point_cloud("cam3_raw.ply",pcd_o3d_cam3)
    # o3d.write_point_cloud("cam3_3h_raw.ply",pcd_o3d_cam3_3h)

    print "written"
    # except : pass




#-----------------------------------




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
    dir = rospy.get_param('~config_dir', os.path.expanduser("~/ws1/catkin_ws/src/finibo_camera_callib/ICP_config"))
    if not os.path.exists(dir):
        os.mkdir(dir)



    pcd_stl = o3d.io.read_point_cloud("/home/finibo/Downloads/calib_cube_rotated_downsampled.ply")
    vol = o3d.visualization.read_selection_polygon_volume("/home/finibo/ws1/libs/test/Open3D/examples/TestData/Crop/cube_cropped.json")

    Visualize=True

    # default_directory=os.path.expanduser("~/ws1/catkin_ws/src/finibo_camera_callib/config")

    rospy.init_node("cam3",anonymous=True)
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)


    # sub1=message_filters.Subscriber("/picker_robot/camera3/depth/color/points",PointCloud2)
    # pcd2_o3d = convertCloudFromRosToOpen3d(sub1)
    # tfBuffer = tf2_ros.Buffer()
    #
    # listener = tf2_ros.TransformListener(tfBuffer)
    # o3d.io.write_point_cloud("cam3_raw.ply", pcd2_o3d)

    queue_size = 10
    fps = .1
    delay = 1 / fps * 0.5

    rospy.Subscriber('/picker_robot/camera3/depth/color/points', PointCloud2, callback)



    rospy.spin()
