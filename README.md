# calibration_ros
Camera Lidar calibration in ROS.

The included cpp code and python2 scripts are built and tested on Ubuntu 18.04 with ROS melodic

### 1. Camera calibration
Given a camera it is important know its parameters (intrinsic - related to internal camera parameters including lense focal lengths in the x,y axes, pixel skew and the optical center; extrinsic - mapping 3D world coordinates on to the 2D image plane; distortion coefficients). More information on these parameters can be found [here](https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html)
There are several techniques to estimate these parameters, in this project a checkerboard approach is used. Mutliple images of a known dimension checkerboard, captured by the camera to be calibrated, are collected. A .bag file with this information is provided. The .bag file can be played back as a "camera" node transmitting Image messages. ROS provides [camera_calibration](http://wiki.ros.org/camera_calibration) package for automatic camera calibration.

#### 1.1. Exploring the .bag file
```bash
$ rosbag info <path-to-bag-file>
```
![alt text](https://github.com/ShanmukhaManoj11/calibration_ros/blob/master/readme_utils/rosbag_info_output.jpg)
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
Image rectification is the process of removing distortions in the image. ROS provides [image_proc](http://wiki.ros.org/image_proc) package that does this processing. The following launch file located at `camera_calibration1/launch/rectified_image_export.launch` is created to play the modified .bag file with correct calibration parameters, rectify the image frames with image_proc/rectify nodelet and save the rectified frames as video.
```xml
<launch>
	<node pkg="nodelet" type="nodelet" name="standalone_nodelet" args="manager"/>

	<node name="rosbag_player" pkg="rosbag" type="play" args="/home/mano/data/2016-11-22-14-32-13_test_modified.bag" required="true"/>

	<node name="rectifyer" pkg="nodelet" type="nodelet" args="load image_proc/rectify standalone_nodelet" required="true">
		<remap from="image_mono" to="sensors/camera/image_color"/>
		<remap from="camera_info" to="/sensors/camera/camera_info"/>
		<remap from="image_rect" to="/sensors/camera/image_rect_color"/>
	</node>

	<node name="rect_video_recorder" pkg="image_view" type="video_recorder" respawn="false">
		<remap from="image" to="/sensors/camera/image_rect_color"/>
	</node>
</launch>
```
sample distorted image             |  rectified image
:---------------------------------:|:---------------------------------:
![alt text](https://github.com/ShanmukhaManoj11/calibration_ros/blob/master/readme_utils/distorted_image.jpg) | ![alt text](https://github.com/ShanmukhaManoj11/calibration_ros/blob/master/readme_utils/rectified_image.jpg) 

**Node graph for the image rectification process**
![alt text](https://github.com/ShanmukhaManoj11/calibration_ros/blob/master/readme_utils/node_graph_image_rectification.jpg)

### 2. Camera Lidar calibration
Given a camera frame and lidar frame, the relative transformation between them is needed for aligning the data points in a single frame of reference for sensor fusion purposes. Given point cloud data, and assuming the extrinsic camera matrix - which describes the rotation and translation of the camera coordinate frame with the world coordinate frame is known, we need to find the transformation matrix that takes the point in the lidar frame and transforms it to the world frame. 
This can be formulated as an optimization problem. A 3D point in world frame represented in homogeneous coordinates as <img src="https://latex.codecogs.com/gif.latex?\begin{bmatrix}&space;x&space;&&space;y&space;&&space;z&space;&&space;1&space;\end{bmatrix}^{T}" title="\begin{bmatrix} x & y & z & 1 \end{bmatrix}^{T}" /> and the corresponding 2D point in image frame represented in homogeneous coordinates as <img src="https://latex.codecogs.com/gif.latex?\begin{bmatrix}&space;u&space;&&space;v&space;&&space;1&space;\end{bmatrix}^{T}" title="\begin{bmatrix} u & v & 1 \end{bmatrix}^{T}" /> are related as follows

<img src="https://latex.codecogs.com/gif.latex?\begin{bmatrix}&space;u&space;&&space;v&space;&&space;1&space;\end{bmatrix}^{T}=K\left&space;[&space;R\mid&space;T&space;\right&space;]\begin{bmatrix}&space;x&space;&&space;y&space;&&space;z&space;&&space;1&space;\end{bmatrix}^{T}" title="\begin{bmatrix} u & v & 1 \end{bmatrix}^{T}=K\left [ R\mid T \right ]\begin{bmatrix} x & y & z & 1 \end{bmatrix}^{T}" />

where `K` is a 3x3 camera intrinsic matrix, `[R|T]` is the 3x4 camera extrinsic matrix (as obtained by the camera calibration process).

And the corresponding point <img src="https://latex.codecogs.com/gif.latex?\begin{bmatrix}&space;X&space;&&space;Y&space;&&space;Z&space;&&space;1&space;\end{bmatrix}^{T}" title="\begin{bmatrix} X & Y & Z & 1 \end{bmatrix}^{T}" /> with homoegeneous representation in the lidar coordinate frame can be transformed to the world frame by multiplying it with the the 4x4 transformation matrix `P` as follows

<img src="https://latex.codecogs.com/gif.latex?\begin{bmatrix}&space;x&space;&&space;y&space;&&space;z&space;&&space;1&space;\end{bmatrix}^{T}=P\begin{bmatrix}&space;X&space;&&space;Y&space;&&space;Z&space;&&space;1&space;\end{bmatrix}^{T}" title="\begin{bmatrix} x & y & z & 1 \end{bmatrix}^{T}=P\begin{bmatrix} X & Y & Z & 1 \end{bmatrix}^{T}" />

Given a set of n 2D-3D correspondence points (2D points obtained from the image, 3D points obtained from the point cloud) our goal is to estimate transformation matrix `P` such that the `L2` error, between the actual 2D points and the estimated 2D points from the product equations described above, is minimized.

#### 2.1. Collecting 2D-3D correspondence points manually
For this project a set of 6 2D-3D correspondence points are collected manually by playing the .bag file and looking at the image frames and the lidar point cloud data in rviz. Launch file image_pointcloud_correspondence.launch shown below located at `camera_lidar_calibration/launch/image_pointcloud_correspondence.launch` is created to start the manual process of collecting the correspondece points
```xml
<launch>
	<node pkg="nodelet" type="nodelet" name="standalone_nodelet" args="manager"/>

	<node name="rosbag_player" pkg="rosbag" type="play" args="/home/mano/data/2016-11-22-14-32-13_test_modified.bag" required="true"/>

	<node name="rectifyer" pkg="nodelet" type="nodelet" args="load image_proc/rectify standalone_nodelet" required="true">
		<remap from="image_mono" to="sensors/camera/image_color"/>
		<remap from="camera_info" to="/sensors/camera/camera_info"/>
		<remap from="image_rect" to="/sensors/camera/image_rect_color"/>
	</node>

	<node name="rqt_image_viewer" pkg="rqt_image_view" type="rqt_image_view" required="true"/>

	<node name="rviz_visualizer" pkg="rviz" type="rviz" args="-f velodyne -d /home/mano/data/rviz_config.rviz"/>
</launch>
```
The launch file runs a node that plays the .bag file with the correct calibration information (that was created as described in section 1.3), an image_proc/rectify nodelet to publish the rectified images, an rqt_image_view node for displaying image frames and rviz displaying point cloud data. 
When an approximate match in the image and the lidar point cloud structure is observed the playback of the bag file is paused and 6 3D points are gathered from rviz over the topic `/clicked_points` by the following command on a terminal window
```
$ rostopic echo /clicked_points > <path-to-txt-file>
```
After the 6 points are selected on the rviz, they get saved in the specified .txt file. After this the corresponding image is saved from the rqt_image_view window for collecting the 2D points. This is done by running the grab_2d_correspondence_points.py script located at `camera_lidar_calibration/scripts/grab_2d_correspondence_points.py` on the command line
```
$ python2 grab_2d_correspondence_points.py -i <path-to-image-saved-from-rqt-image-view-window>
```
Points from the image can be collected by mouse clicks. After the 6 appropriate points are selected, pressing the key 'c' would close the application and the selected points are displayed on the screen.
The 3D points collected from rviz saved to a specified .txt file and the 2D points collected from image displayed on the terminal window are copied to a json file. The json file created for this project can be found at `utils/camera_lidar_calibration_utils/correspondence_points_2.json` and following are the contents of that file.
```json
{
	"points": [ 
		[ 2.946, 0.877, -0.269, 1.0 ], 
		[ 1.545, 0.156, -0.082, 1.0 ], 
		[ 1.539, -0.377, -0.083, 1.0 ], 
		[ 1.580, -0.372, -0.376, 1.0 ], 
		[ 1.592, 0.167, -0.372, 1.0 ], 
		[ 1.730, -0.175, 0.152, 1.0 ] 
	],
	"uvs": [
		[ 252, 404 ], 
		[ 317, 323 ],
		[ 488, 330 ],
		[ 484, 436 ],
		[ 308, 426 ],
		[ 424, 286 ]
	],
	"initialTransform": [ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 ],
	"bounds": [
		[ -4, 4 ], 
		[ -4, 4 ], 
		[ -4, 4 ], 
		[ 0, 6.283 ],
		[ 0, 6.283 ],
		[ 0, 6.283 ]
	]
}
```
The `points` field in the above file has the homegeneous representation of 6 3D points, `uvs` field has the 2D points. The `initialTransform` is the initial state and `bounds` is the bounds on the state vetor fed as inputs to the optimization application.

#### 2.2. Estimating the lidar-to-world transformation matrix
As explained above, one approach to estimating this transformation matrix is solving an optimization problem. Since tackling an **optimization** problem in python is a little simpler than that in c++ (thanks to scipy), a python script to estimate the transformation is created. It is located at `camera_lidar_calibration/scripts/camera_lidar_calibration.py`.

Procedure to run the camera_lidar_calibration.py script

**Terminal 1**
```
$ roscore
```
**Terminal 2**
```
$ rosbag play <path-to-bag-file-with-correct-camera-calibration-information>
```
**Terminal 3**
```
$ rosrun camera_lidar_calibration camera_lidar_calibration.py -c <path-to-correspondence-points-json-file> -i <path-to-image-saved-for-collecting-2D-correspondence-points>
```

After the solution converge, on **Terminal 2** the result is displayed. The result is list of 6 floats representing the following respectively `[translation x, translation y, translation z, yaw, pitch, roll]`

The final transform obtained from the correspondence_point_2.json included in the project is

`[-0.269326290 -0.450351774 -0.219415375 1.62507734 4.87444952 0.0]`

### 3. Fusing image and point cloud data
The final transform achieved earlier can be expanded to from the transformation matrix `P` described in section 2. ROS has tf package that deals with the transform messages. This final transform will be pusblished as a static_transform from `frame_id:"velodyne"` (source frame/ lidar frame) to `frame_id:"world"` (target frame/ world frame). `TransformListener` in ROS can listen for this transform message and by using provided APIs like `transformPoint(...)` the points in frame_id:"velodyne" can be converted to frame_id:"world". These converted points in world frame are transformed to image points using `PinholeCameraModel::project3dToPixel(...)`. This transformation of point cloud data is implemented in camera_lidar_overlay.cpp located at `camera_lidar_calibration/src/camera_lidar_overlay.cpp`.

The following launch file located at `camera_lidar_calibration/launch/image_pointcloud_overlay.launch` is created to run 
1. node that playback the bag file with correct camera calibration information,
2. image_proc/rectifier nodelet for rectifying the images,
3. static_transform_publisher publishing the world-to-lidar transformation parameters from the previous section,
4. image_view video_recorder node recording the point-cloud overlayed image frames into a video, and
5. rviz to visualize image overlayed point-cloud data 
```xml
<launch>
	<node pkg="nodelet" type="nodelet" name="standalone_nodelet" args="manager"/>

	<node name="rosbag_player" pkg="rosbag" type="play" args="/home/mano/data/2016-11-22-14-32-13_test_modified.bag" required="true"/>

	<node name="rectifyer" pkg="nodelet" type="nodelet" args="load image_proc/rectify standalone_nodelet" required="true">
		<remap from="image_mono" to="sensors/camera/image_color"/>
		<remap from="camera_info" to="/sensors/camera/camera_info"/>
		<remap from="image_rect" to="/sensors/camera/image_rect_color"/>
	</node>

	<node name="static_transform_publisher" pkg="tf" type="static_transform_publisher" args="-0.269326290 -0.450351774 -0.219415375 1.62507734 4.87444952 0.0 world velodyne 100" required="true"/>

	<node name="image_pointcloud_overlay" pkg="camera_lidar_calibration" type="camera_lidar_overlay"/>

	<node name="video_recorder" pkg="image_view" type="video_recorder" respawn="false">
		<remap from="image" to="/sensors/camera/overlayed_lidar_image"/>
	</node>

	<node name="rviz_visualizer" pkg="rviz" type="rviz" args="-f velodyne -d /home/mano/data/lidar_image_overlay.rviz"/>
</launch>
```
**Node graph for image-pointcloud fusion**
![alt text](https://github.com/ShanmukhaManoj11/calibration_ros/blob/master/readme_utils/node_graph_image_lidar_overlay.jpg)

#### 3.1. Overlaying point cloud data on image
After transforming the points in the cloud to the corresponding image points, they can be added on to the image using OpenCV drawing functions. The overlayed image is published by the camera_lidar_overlay node on the topic `/sensors/camera/overlayed_lidar_image`

**Sample point cloud overlayed image**
![alt text](https://github.com/ShanmukhaManoj11/calibration_ros/blob/master/readme_utils/point_cloud_on_image.jpg)

#### 3.2 Overlaying RGB image on point cloud data
The transfromed points in the cloud correspond to some pixel location on the image. The RGB values at that pixel location are collected and for each point in the cloud a new point of type `pcl::PointXYZRGB` is created with x,y,z values set to be that of the original point and the r,g,b values assigned to be the collected RGB values. These newly formed pcl::PointXYZRGB points are converted to ROS message and published on the topic `/image_fused_pointcloud`

**Sample image overlayed point cloud**
![alt text](https://github.com/ShanmukhaManoj11/calibration_ros/blob/master/readme_utils/image_on_point_cloud.jpg)

### Results
1. [image rectification with ROS image_proc](https://youtu.be/yC97zbxJZFU)
2. [point cloud data overlayed on image frames](https://youtu.be/spoKaul9xHE)
3. [RGB image overlayed on point cloud data](https://youtu.be/0MZzsvB2uBQ)

### References
1. [LiDAR-Camera Calibration using 3D-3D Point correspondences](https://arxiv.org/pdf/1705.09785.pdf)
2. https://github.com/swyphcosmo/ros-camera-lidar-calibration
