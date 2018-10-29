#!/usr/bin/env python2
PACKAGE='camera_lidar_calibration'
import roslib; roslib.load_manifest(PACKAGE)
import cv2
import cv_bridge
from image_geometry import PinholeCameraModel
import tf
import rospy
import sys
from sensor_msgs.msg import Image,CameraInfo
import numpy as np
from scipy.optimize import minimize
import math
import random
import demjson
import time
import argparse
import pickle

settings={}
cameraModel=PinholeCameraModel()
cameraInfo=CameraInfo()
camera={}

def costFunction(x):
	global cameraModel,settings
	# ( tx, ty, tz, Ay, Ap, Ar )
	# format for static transform publisher
	parameters=x
	T=[parameters[0],parameters[1],parameters[2],1.0]
	# euler_matrix( roll, pitch, yaw )
	R=tf.transformations.euler_matrix(parameters[5],parameters[4],parameters[3])
	#print T,R
	R[:,3]=T
	error=0
	for i in range(0,len(settings['points'])):
		point=settings['points'][i]
		expected_uv=settings['uvs'][i]
		rotatedPoint=R.dot(point)
		uv=cameraModel.project3dToPixel(rotatedPoint)
		diff=np.array(uv)-np.array(expected_uv)
		error=error+np.sqrt(np.sum(diff ** 2))
	return error

def cameraCallback(data):
	global settings,cameraModel,camera
	cameraInfo=data
	print 'Received Camera Info'
	camera.unregister()
	cameraModel.fromCameraInfo(cameraInfo)
	print('Starting optimization')
	result=minimize(costFunction,settings['initialTransform'],bounds=settings['bounds'],method='SLSQP',options={'disp':True,'maxiter':1000})
	while not result.success or result.fun>50:
		for i in range(0,len(settings['initialTransform'])):
			settings['initialTransform'][i]=random.uniform(settings['bounds'][i][0],settings['bounds'][i][1])
		print 'Trying new starting point:' 
		print settings['initialTransform']
		result=minimize(costFunction,settings['initialTransform'],bounds=settings['bounds'],method='SLSQP',options={'disp':True,'maxiter':1000})

	print 'Finished calibration'
	print 'Final transform:'
	print result.x 
	print 'Error: ',str(result.fun)
	with open('./static_transform.pickle','w') as pfile:
		pickle.dump(result.x,pfile,-1)
	print 'saved static_transform in ./static_transform.pickle file'

	sys.stdout.flush()
	rospy.signal_shutdown('Finished calibration')

def generateImage(parameters,cameraModel,infile,outfile):
	image=cv2.imread(infile)
	# parameters = ( tx, ty, tz, Ay, Ap, Ar )
	# format for static transform publisher
	T=[parameters[0],parameters[1],parameters[2],1.0]
	# euler_matrix( roll, pitch, yaw )
	R=tf.transformations.euler_matrix(parameters[5],parameters[4],parameters[3])
	R[:,3]=T
	for i in range(0,len(settings['points'])):
		point=settings['points'][i]
		expected_uv=settings['uvs'][i]
		rotatedPoint=R.dot(point)
		uv=cameraModel.project3dToPixel(rotatedPoint)
		cv2.circle(image,(int(expected_uv[0]),int(expected_uv[1])),5,(255,0,0),thickness=-1)
		cv2.circle(image,(int(uv[0]),int(uv[1])),5,(0,0,255),thickness=-1)
	cv2.imwrite(outfile,image)

if __name__ == '__main__':
	parser=argparse.ArgumentParser()
	parser.add_argument("-c","--config",required=True,help="Path to the config file with correspondence points")
	parser.add_argument("-i","--image",help="Path to the image from which 2D correspondence points were grabbed")
	parser.add_argument("-o","--out",help="Path to the save image overlayed with actual and estimated 2D correspondence points after calibration")
	args=parser.parse_args()

	settings_file=args.config
	settings=demjson.decode_file(settings_file)
	# check settings
	if not settings['bounds']:
		print 'Parameter `bounds` not found in `',settings_file,'`'
		print 'Using default bounds'
		settings['bounds']=[[-5,5],[-5,5],[-5,5],[0,6.283],[0,6.283],[0,6.283]]
	if not settings['initialTransform']:
		print 'Parameter `initialTransform` not found in `',settings_file,'`'
		print 'Using default starting point'
		settings['initialTransform']=[0,0,0,0,0,0]
	if not settings['points'] or not settings['uvs']:
		print 'Point correspondences not found in `',settings_file,'`'
		print 'Both `points` and `uvs` are required'
		sys.exit(2)

	try:
		rospy.init_node('lidar_image_calibration')
		camera=rospy.Subscriber('/sensors/camera/camera_info',CameraInfo,callback=cameraCallback,queue_size=1)
		rospy.spin()
	except rospy.ROSInterruptException:
		pass

	if args.image is not None:
		print 'Generating image from',args.image
		with open('./static_transform.pickle','r') as pfile:
			static_transform=pickle.load(pfile)
		infile=args.image
		if args.out is not None:
			outfile=args.out
		else:
			outfile=args.image[0:len(args.image)-4]+'_generated.jpg'
		generateImage(static_transform,cameraModel,infile,outfile)
		print 'Generated image at',outfile
