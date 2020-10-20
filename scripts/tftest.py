#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
# import message_filters
# from std_msgs.msg import String
# import sensor_msgs.point_cloud2 as pc2
# from sensor_msgs.msg import PointCloud2, PointField, CameraInfo
# import time
# import pickle
import tf
import numpy as np
# from ros_numpy import numpify
# from ros_numpy.point_cloud2 import split_rgb_field


if __name__ == '__main__':
    rospy.init_node('tf_turtle')
    listener = tf.TransformListener()
    # rospy.wait_for_service('spawn')
    # spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
    # spawner(4, 2, 0, 'turtle2')
    # turtle_vel = rospy.Publisher('turtle2/cmd_vel', geometry_msgs.msg.Twist


    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('camera3_depth_optical_frame', 'camera3_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        print("trans",trans)
        print("rot",rot)
        rate.sleep()