#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"

class velodyneInfoSubscriber{
private:
	ros::NodeHandle nh;
	ros::Subscriber velodyne_info_sub;
public:
	velodyneInfoSubscriber();
	void callback(const sensor_msgs::PointCloud2ConstPtr& msg);
};

velodyneInfoSubscriber::velodyneInfoSubscriber():
velodyne_info_sub(nh.subscribe<sensor_msgs::PointCloud2>("/sensors/velodyne_points",1,&velodyneInfoSubscriber::callback,this)){
}
void velodyneInfoSubscriber::callback(const sensor_msgs::PointCloud2ConstPtr& msg){
	ROS_INFO("height: %d,width: %d,point_step: %d,row_step: %d,is_bigendian: %d,is_dense: %d",msg->height,msg->width,msg->point_step,msg->row_step,msg->is_bigendian,msg->is_dense);
	std::cout<<msg->header<<std::endl;
	pcl::PointCloud<pcl::PointXYZ> cloud;
	pcl::fromROSMsg(*msg,cloud);
	std::cout<<cloud<<std::endl;
}

int main(int argc,char **argv){
	ros::init(argc,argv,"velodyne_info_subscriber");
	velodyneInfoSubscriber velodyne_info_subscriber;
	ros::spin();
	return 0;
}