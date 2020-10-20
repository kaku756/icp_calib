#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import open3d as o3d



Tbh=np.array([[ 0.07716927 , 0.10071757 ,-0.99191778 , 0.25755102]
 [ 0.02542272 ,-0.99475981 ,-0.09902831 , 0.01798585]
 [-0.99669383 ,-0.01757531 ,-0.0793254 ,  1.12117939]
 [ 0.     ,     0.     ,     0.     ,     1.        ]])


Thb=np.linalg.inv(Tbh)

