#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import os
import message_filters
import open3d as o3d

pcd=o3d.read_point_cloud("cam3_3h_raw.ply")

vol = o3d.visualization.read_selection_polygon_volume("/home/finibo/ws1/libs/test/Open3D/examples/TestData/Crop/cube_cropped.json")

cropped = vol.crop_point_cloud(pcd)

o3d.io.write_point_cloud("cropped_3h.ply",cropped)

o3d.visualization.draw_geometries([cropped])

cropped.transform(Th3)


