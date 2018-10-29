# calibration_ros
Camera Lidar calibration in ROS

### 1. Camera calibration
Given a camera it is important know its parameters (intrinsic - related to internal camera parameters including lense focal lengths in the x,y axes, pixel skew and the optical center; extrinsic - mapping 3D world coordinates on to the 2D image plane; distortion coefficients). More information on these parameters can be found [here](https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html)
There are several techniques to estimate these parameters, in this project a checkerboard approach is used. Mutliple images of a known dimension checkerboard, captured by the camera to be calibrated, are collected. A .bag file with this information is provided. The .bag file can be played back as a "camera" node transmitting Image messages. ROS provides [camera_calibration](http://wiki.ros.org/camera_calibration) package for automatic camera calibration.

#### 1.1. Exploring the .bag file
```bash
$ rosbag info <path-to-bag-file>
```
The provided .bag file publishes 
1. raw images of message type `sensor_msgs::Image` on the topic `/sensors/camera/image_color`
2. calibration information of message type `sensor_msgs::CameraInfo` on the topic `/sensors/camera/camera_info`
3. lidar data of message type `sensor_msgs::PointCloud2` on the topic `/sensors/velodyne_points`

#### 1.2. camera_calibration1 ROS package
camera_calibration1 packge contains the following launch file located at `camera_calibration1/launch/camera_calibration1.launch` that creates a node to playback the .bag file that publishes the images and a node for automatic calibration using cameracalibrator.py of the ROS's camera_calibration package that subscribes
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
The checkerboard used in the images has 5x7 corners and each square is of size 5 cm. 
```
$ roslaunch camera_calibration1 camera_calibration1.launch
```
This command opens a gui window displaying the images with detected corners and also the progress of the calibration process. Once the process is finished the calibration information can be created by cliking the `calibrate` button on the gui and `save` button saves the calibration data as .yaml file along with the image frames the cameracalibrator.py has used to generate this calibration. By default this data was stored as a compressed .tar.gz file in the .ros directory which was copied to a different location. The data is found at `utils/camera_calibrationdata/` and the following ost.yaml contains the generated calibration parameters.
```yaml
image_width: 964
image_height: 724
camera_name: narrow_stereo
camera_matrix:
  rows: 3
  cols: 3
  data: [485.070038, 0.000000, 457.193914, 0.000000, 485.421504, 365.293827, 0.000000, 0.000000, 1.000000]
distortion_model: plumb_bob
distortion_coefficients:
  rows: 1
  cols: 5
  data: [-0.200465, 0.069475, 0.003302, 0.000217, 0.000000]
rectification_matrix:
  rows: 3
  cols: 3
  data: [1.000000, 0.000000, 0.000000, 0.000000, 1.000000, 0.000000, 0.000000, 0.000000, 1.000000]
projection_matrix:
  rows: 3
  cols: 4
  data: [427.078461, 0.000000, 461.243076, 0.000000, 0.000000, 433.646820, 369.922391, 0.000000, 0.000000, 0.000000, 1.000000, 0.000000]
```

#### 1.3. Modifying the camera_info content in the existing bag file with the generated calibration information
Initial approach was to create a camera_info_publisher.cpp which acts as a node that subcribers to the `/sensors/camera/camera_info` topic and replaces the content of the received `sensor_msgs::CameraInfo` message with the contents in the ost.yaml file. This implementation can be found at `camera_calibration1/src/camera_info_publisher.cpp`
This approach would require this node to be running along with the rosbag player whenever the calibration info is needed by other applications.
The second approach is to modify the .bag file itself and create a new one with the camera_info topic contents modified. This approach is implemented in the modify_camera_info_content.py script found at `camera_calibration1/scripts/modify_camera_info_content.py`. Now this script only needs to be run once and this would create a new .bag file that can be played back with the correct calibration information to be used by other applications. 

#### 1.4 Rectifying images
Image rectification is the process of removing distortions in the image. ROS provide [image_proc](http://wiki.ros.org/image_proc) package that does this processing.

### 2. Camera Lidar calibration
Given a camera frame and lidar frame, the relative transformation between them is needed for aligning the data points in a single frame of reference for sensor fusion purposes. Given point cloud data, and assuming the camera coordinate frame is aligned with the world frame, we need to find the transformation that takes the point in the lidar frame to the world frame. 
This can be formulated as an optimization problem. A point <img src="https://latex.codecogs.com/svg.latex?\Large&space;x=\frac{-b\pm\sqrt{b^2-4ac}}{2a}" title="\Large x=\frac{-b\pm\sqrt{b^2-4ac}}{2a}" />
