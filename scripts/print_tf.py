import rospy
import roslib
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
import tf2_ros
# import tf


if __name__ == "__main__":

    rospy.init_node('main', anonymous=True) #make node
    # rospy.Subscriber("/scan", LaserScan, scanCb,queue_size = 1)
    rate = rospy.Rate(5) # 5hz
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    transform = tf_buffer.lookup_transform("workspace_link", "camera4_link", rospy.Time())


    listener = tf.TransformListener()
    while not rospy.is_shutdown():

        trans=listener.waitForTransform("workspace_link", "camera4_link", rospy.Time(0),rospy.Duration(1.0))
        # p=listener.transformPointCloud("odom",points2d.base_link_point2d)
        rate.sleep()