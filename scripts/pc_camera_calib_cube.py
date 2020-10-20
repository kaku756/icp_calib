#!/usr/bin/env python
# -*- coding: utf-8 -*-

import angleAxis_to_rpy
import math
import numpy as np
from lib_conversion_between_transform_and_quaternion import convertRostfTotransform, convert_pq_to_Transform
import rospy
import tf2_ros
import open3d as o3d
import copy
from open3d.open3d.geometry import voxel_down_sample,estimate_normals


def draw_registration_result(source, target, transformation):
 source_temp = copy.deepcopy(source)
 target_temp = copy.deepcopy(target)
 source_temp.paint_uniform_color([1, 0.706, 0])
 target_temp.paint_uniform_color([0, 0.651, 0.929])
 source_temp.transform(transformation)
 o3d.visualization.draw_geometries([source_temp, target_temp])



rx=math.radians(-96.30853)
ry=math.radians(43.91451)
rz=math.radians(91.32387)

x=339.3502/1000
y=26.15644/1000
z=1127.767/100

rot=angleAxis_to_rpy.rpy_to_rot([rx,ry,rz])
transform=[x,y,z]

T=convert_pq_to_Transform(transform,rot)        #base_link to TCP
print T


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


                                                #tfでpulishされているのと比べる
if __name__=="__main__":


    rospy.init_node("subscribe_hand_link_tf")
    tf_Buffer = tf2_ros.Buffer()
    listener=tf2_ros.TransformListener(tf_Buffer)

    rate = rospy.Rate(10.0)
    num_recieved_tf=0
    while not rospy.is_shutdown():
        try:
            transform=tf_Buffer.lookup_transform("base_link","hand_link",rospy.Time())
            # transform=tf_Buffer.lookup_transform("workspace_link", "camera3_depth_optical_frame",rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):

            rate.sleep()
            continue

        p2, q2 = transform_to_pq(transform)  # T4b
        Tbh = convertRostfTotransform(p2, q2)
        num_recieved_tf+=1
        if num_recieved_tf>=10:
            break
        # print transform
        # print Tbh
        rate.sleep()











