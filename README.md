# calibration_ros
Camera Lidar calibration in ROS

## Camera calibration
Given a camera it is important know its parameters (intrinsic - related to internal camera parameters including lense focal lengths in the x,y axes, pixel skew and the optical center; extrinsic - mapping 3D world coordinates on to the 2D image plane; distortion coefficients). More information on these parameters can be found [here](https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html)

There are several techniques to estimate these parameters, in this project a checkerboard approach is used. Mutliple images of a known dimension checkerboard, captured by the camera to be calibrated, are collected. A .bag file with this information is provided. The .bag file can be played back as a "camera" node transmitting Image messages. ROS provides [camera_calibration](http://wiki.ros.org/camera_calibration) package for automatic camera calibration.

### Exploring the .bag file
```console
$ rosbag info <path-to-bag-file>
```

The provided .bag file publishes 
1. raw images of message type `sensor_msgs::Image` on the topic `/sensors/camera/image_color`
2. calibration information of message type `sensor_msgs::CameraInfo` on the topic `/sensors/camera/camera_info`
3. lidar data of message type `sensor_msgs::PointCloud2` on the topic `/sensors/velodyne_points`

### camera_calibration1 ROS package
camera_calibration1 packge contains the following launch file located at `camera_calibration1/launch/camera_calibration1.launch` that creates a node to playback the .bag file and a node for automatic calibration using cameracalibrator.py of the ROS's camera_calibration package that subscribes

```xml
<launch>
	<node 
		name="calibrator" 
		pkg="camera_calibration" 
		type="cameracalibrator.py" 
		args="--size 5x7 --square 0.05 image:=/sensors/camera/image_color" />
	<node
		name="rosbag_player"
		pkg="rosbag"
		type="play"
		args="-r 0.5 /home/mano/data/2016-11-22-14-32-13_test.bag" />
</launch>
```
