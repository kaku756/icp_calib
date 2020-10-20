#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import os
import message_filters
import open3d as o3d
import numpy as np

T=np.eye(4)
T[0,3]=-1
pcd=o3d.io.read_point_cloud("cam3_crop.ply")
pcd.transform(T)
o3d.io.write_point_cloud("testminusx.ply",pcd)