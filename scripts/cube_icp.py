#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
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


Tbh=np.array([[ 0.07716927 , 0.10071757 ,-0.99191778 , 0.25755102],
 [ 0.02542272 ,-0.99475981 ,-0.09902831 , 0.01798585],
 [-0.99669383 ,-0.01757531 ,-0.0793254 ,  1.12117939],
 [ 0.     ,     0.     ,     0.     ,     1.        ]])



Thb=np.linalg.inv(Tbh)

# print np.dot(Tbh,Thb)
pcd = o3d.io.read_point_cloud("/home/finibo/Downloads/calib_cube_rotated_downsampled.ply")

cam3_raw=o3d.io.read_point_cloud("cam3.ply")
cam3_hand=copy.deepcopy(cam3_raw).transform(Tbh)
o3d.io.write_point_cloud("cam3_hand.ply",cam3_hand)

cam3pcd = o3d.io.read_point_cloud("cam3_crop.ply")

o3d.visualization.draw_geometries([cam3_raw,cam3_hand])


T = np.eye(4)
T[2,3] = 1
# T_test=np.array([[ 1 , 0.0 ,-0.99191778 , 100.25755102],
#  [ 0.02542272 ,-0.99475981 ,-0.09902831 , 0.01798585],
#  [-0.99669383 ,-0.01757531 ,-0.0793254 ,  1.12117939],
#  [ 0.     ,     0.     ,     0.     ,     1.        ]])


source=cam3pcd
target=pcd

estimate_normals(cloud=source, search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.01, max_nn=30))
estimate_normals(cloud=target, search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.01, max_nn=30))

threshold = 0.02

trans_init=Thb

print "init Thb"
print Thb

draw_registration_result(source, target, trans_init)
print("Initial alignment")
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
print("")
draw_registration_result(source, target, reg_p2p.transformation)

print("Apply point-to-plane ICP")
reg_p2l = o3d.registration.registration_icp(
 source, target, threshold, trans_init,
 o3d.registration.TransformationEstimationPointToPlane())
print(reg_p2l)
print("Transformation is:")
print(reg_p2l.transformation)

# dict=t2q(reg_p2l.transformation)

print("")
draw_registration_result(source, target, reg_p2l.transformation)

# o3d.io.write_point_cloud("cube_stl_use_this_test.ply",target)

# source=
