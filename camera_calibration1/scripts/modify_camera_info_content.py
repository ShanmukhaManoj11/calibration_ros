#!/usr/bin/env python2

PACKAGE='camera_calibration1'
import roslib; roslib.load_manifest(PACKAGE)
import rospy
import rosbag
import yaml
import os
import sys
import argparse

with open('/home/mano/data/calibrationdata/ost.yaml','r') as f:
	calibration_data=yaml.load(f)

in_bag=rosbag.Bag('/home/mano/data/2016-11-22-14-32-13_test.bag','r')
out_bag=rosbag.Bag('/home/mano/data/2016-11-22-14-32-13_test_modified1.bag','w')

for topic,msg,t in in_bag.read_messages():
	if topic=='/sensors/camera/camera_info':
		modified_msg=msg
		modified_msg.width=calibration_data['image_width']
		modified_msg.height=calibration_data['image_height']
		modified_msg.K=calibration_data['camera_matrix']['data']
		modified_msg.D=calibration_data['distortion_coefficients']['data']
		modified_msg.R=calibration_data['rectification_matrix']['data']
		modified_msg.P=calibration_data['projection_matrix']['data']
		modified_msg.distortion_model=calibration_data['distortion_model']
		msg=modified_msg
	out_bag.write(topic,msg,t)

in_bag.close()
out_bag.close()


