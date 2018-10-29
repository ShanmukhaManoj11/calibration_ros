#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"
#include "tf/transform_listener.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "image_transport/image_transport.h"
#include "opencv2/highgui/highgui.hpp"
#include "cv_bridge/cv_bridge.h"
#include "image_geometry/pinhole_camera_model.h"
#include "geometry_msgs/PointStamped.h"
#include <vector>

class cameraLidarOverlay{
private:
	ros::NodeHandle nh;
	image_transport::ImageTransport it;
	image_transport::Subscriber image_sub;
	image_transport::Publisher overlayed_image_pub;
	ros::Subscriber velodyne_info_sub;
	ros::Subscriber camera_info_sub;
	tf::TransformListener tf_listener;
	tf::StampedTransform transform;
	pcl::PointCloud<pcl::PointXYZ> velodyne_pc;
	image_geometry::PinholeCameraModel cam_model;
	cv::Mat image;
	ros::Publisher image_fused_pcl_pub;
	std::vector<geometry_msgs::PointStamped> transform_point_cloud();
public:
	cameraLidarOverlay();
	void lidar_callback(const sensor_msgs::PointCloud2ConstPtr& msg);
	void image_callback(const sensor_msgs::ImageConstPtr& msg);
	void camera_info_callback(const sensor_msgs::CameraInfoConstPtr& msg);
};

cameraLidarOverlay::cameraLidarOverlay():
it(nh),
velodyne_info_sub(nh.subscribe<sensor_msgs::PointCloud2>("/sensors/velodyne_points",1,&cameraLidarOverlay::lidar_callback,this)),
image_sub(it.subscribe("/sensors/camera/image_rect_color",1,&cameraLidarOverlay::image_callback,this)),
camera_info_sub(nh.subscribe<sensor_msgs::CameraInfo>("/sensors/camera/camera_info",1,&cameraLidarOverlay::camera_info_callback,this)),
overlayed_image_pub(it.advertise("/sensors/camera/overlayed_lidar_image",1)),
image_fused_pcl_pub(nh.advertise<sensor_msgs::PointCloud2>("/image_fused_pointcloud",1)){
}

std::vector<geometry_msgs::PointStamped> cameraLidarOverlay::transform_point_cloud(){
	std::vector<geometry_msgs::PointStamped> transformed_pt_vector;
	geometry_msgs::PointStamped pt, transformed_pt;
  	for(size_t i=0;i<velodyne_pc.points.size();i++){
  		pt.header.frame_id="velodyne";
  		pt.header.stamp=ros::Time();
	  	pt.point.x=velodyne_pc.points[i].x;
	  	pt.point.y=velodyne_pc.points[i].y;
	  	pt.point.z=velodyne_pc.points[i].z;
	  	tf_listener.transformPoint("world",pt,transformed_pt);
	  	//std::cout<<pt<<std::endl;
	  	//std::cout<<transformed_pt<<std::endl;
	  	transformed_pt_vector.push_back(transformed_pt);
  	}
  	return transformed_pt_vector;
}

void cameraLidarOverlay::lidar_callback(const sensor_msgs::PointCloud2ConstPtr& msg){
	//ROS_INFO("height: %d,width: %d,point_step: %d,row_step: %d,is_bigendian: %d,is_dense: %d",msg->height,msg->width,msg->point_step,msg->row_step,msg->is_bigendian,msg->is_dense);
	//std::cout<<msg->header<<std::endl;
	pcl::fromROSMsg(*msg,this->velodyne_pc);
	//std::cout<<(this->velodyne_pc)<<std::endl;
}

//ref: http://wiki.ros.org/image_transport/Tutorials/SubscribingToImages
//http://wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20listener%20%28C%2B%2B%29
void cameraLidarOverlay::image_callback(const sensor_msgs::ImageConstPtr& msg){
  	int width=msg->width, height=msg->height;
  	try{
  		this->image=cv_bridge::toCvShare(msg,"bgr8")->image;
  	}
  	catch(cv_bridge::Exception& e){
  		ROS_ERROR("Could not convert from '%s' to 'bgr8'.",msg->encoding.c_str());
  	}
  	cv::Mat image_copy=(this->image).clone();
  	pcl::PointCloud<pcl::PointXYZ> transformed_velodyne_pc;
  	try{
  		tf_listener.waitForTransform("world","velodyne",ros::Time(0),ros::Duration(10.0));
  		//tf_listener.lookupTransform("world","velodyne",ros::Time(0),transform);
  	}
  	catch(tf::TransformException e){
  		ROS_ERROR("%s",e.what());
  	}
  	//std::cout<<transform.getOrigin().x()<<" "<<transform.getOrigin().y()<<" "<<transform.getOrigin().z()<<std::endl;
  	pcl::PointCloud<pcl::PointXYZRGB> image_fused_pc;
  	pcl::PointXYZRGB image_fused_pt;
  	std::vector<geometry_msgs::PointStamped> transformed_pt_vector=transform_point_cloud();
  	for(auto transformed_pt: transformed_pt_vector){
  		double x=transformed_pt.point.x, y=transformed_pt.point.y, z=transformed_pt.point.z;
  		/*if(sqrt(x*x+y*y+z*z)>5){
  			continue;
  		}
  		else{*/
		cv::Point3d xyz(x,y,z);
		cv::Point2d uv=cam_model.project3dToPixel(xyz);
		if(uv.x>=0 && uv.x<width && uv.y>=0 && uv.y<height){
			cv::circle(this->image,uv,3.0,cv::Scalar(0,255,0));
			image_fused_pt.x=x;
			image_fused_pt.y=y;
			image_fused_pt.z=z;
			cv::Vec3b intensity=image_copy.at<cv::Vec3b>(int(uv.y),int(uv.x));
			image_fused_pt.b=intensity.val[0];
			image_fused_pt.g=intensity.val[1];
			image_fused_pt.r=intensity.val[2];
			image_fused_pc.push_back(image_fused_pt);
		}
  		//}
  	}
  	sensor_msgs::ImagePtr pub_msg=cv_bridge::CvImage(std_msgs::Header(),"bgr8",this->image).toImageMsg();
  	overlayed_image_pub.publish(pub_msg);
  	sensor_msgs::PointCloud2 image_fused_pc_msg;
  	pcl::toROSMsg(image_fused_pc,image_fused_pc_msg);
  	image_fused_pc_msg.header.stamp=ros::Time::now();
  	image_fused_pc_msg.header.frame_id="world";
  	image_fused_pcl_pub.publish(image_fused_pc_msg);
  	cv::imshow("view",this->image);
	cv::waitKey(30);
}

void cameraLidarOverlay::camera_info_callback(const sensor_msgs::CameraInfoConstPtr& msg){
	//std::cout<<msg<<std::endl;
	cam_model.fromCameraInfo(msg);
	//ROS_INFO("camera model set from camera info");
}

int main(int argc,char **argv){
	cv::namedWindow("view");
	ros::init(argc,argv,"camera_lidar_overlay");
	cameraLidarOverlay camera_lidar_overlay;
	ros::spin();
	cv::destroyWindow("view");
	return 0;
}