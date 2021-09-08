#!/usr/bin/env python
import cv2
from cv_bridge import CvBridge
import numpy as np

import rospy
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import String, Header
import sensor_msgs.point_cloud2 as pcl2

import os
DATA_PATH = '/home/anny/kittiRawData/2011_09_26/2011_09_26_drive_0005_sync/'

if __name__ == '__main__':
    
	rospy.init_node('kitti_node',anonymous=True)
	camera_pub = rospy.Publisher('kitti_cam', Image, queue_size=10)
	pcl_pub = rospy.Publisher('kitti_point_cloud', PointCloud2, queue_size=10)
	bridge = CvBridge()
	frame = 0	

	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		img = cv2.imread(os.path.join(DATA_PATH, 'image_02/data/%010d.png'%frame))
		point_cloud = np.fromfile(os.path.join(DATA_PATH, 'velodyne_points/data/%010d.bin'%frame), dtype=np.float32).reshape(-1,4)
		camera_pub.publish(bridge.cv2_to_imgmsg(img, "bgr8"))
		rospy.loginfo("camera image published")
		header = Header()
		header.stamp = rospy.Time.now()
		header.frame_id = 'map'
		pcl_pub.publish(pcl2.create_cloud_xyz32(header, point_cloud[:,:3]))
		rospy.loginfo("pcl published")
		rate.sleep()
		frame += 1
		frame %= 154
