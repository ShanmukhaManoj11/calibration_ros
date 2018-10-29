#include "ros/ros.h"
#include "sensor_msgs/CameraInfo.h"
#include "std_msgs/UInt32.h"
#include "camera_calibration_parsers/parse.h"
#include <boost/program_options.hpp>

class cameraInfoPublisher{
private:
	ros::NodeHandle nh;
	ros::Subscriber camera_info_sub;
	ros::Publisher camera_info_pub;
	sensor_msgs::CameraInfo modified_msg;
public:
	cameraInfoPublisher(const std::string& file_name,std::string& camera_name);
	void modify_received_msg(const sensor_msgs::CameraInfoConstPtr& msg);
};

cameraInfoPublisher::cameraInfoPublisher(const std::string& file_name,std::string& camera_name):
camera_info_sub(nh.subscribe<sensor_msgs::CameraInfo>("/sensors/camera/camera_info",1,&cameraInfoPublisher::modify_received_msg,this)),
camera_info_pub(nh.advertise<sensor_msgs::CameraInfo>(camera_name+"/camera_info",1)){
	bool read_status=camera_calibration_parsers::readCalibration(file_name,camera_name,modified_msg);
	if(read_status){
		ROS_INFO("done reading calibration file");
		//ROS_INFO("publishing calibration data on %s/camera_info",camera_name.c_str());
		std::cout<<"publishing calibration data on "<<camera_name<<"/camera_info"<<std::endl;
	}
	else{
		ROS_INFO("error reading calibration file");
	}
}
void cameraInfoPublisher::modify_received_msg(const sensor_msgs::CameraInfoConstPtr& msg){
	modified_msg.header=msg->header;
	/*modified_msg.height=msg->height;
	modified_msg.width=msg->width;
	modified_msg.distortion_model=msg->distortion_model;
	modified_msg.D=msg->D;
	modified_msg.K=msg->K;
	modified_msg.R=msg->R;
	modified_msg.P=msg->P;*/
	modified_msg.binning_x=msg->binning_x;
	modified_msg.binning_y=msg->binning_y;
	modified_msg.roi=msg->roi;
	camera_info_pub.publish(modified_msg);
}

int main(int argc,char **argv){
	std::string file_name;//="/home/mano/data/calibrationdata/ost.yaml";
	std::string camera_name;//="/calibrated_camera";

	namespace po=boost::program_options;
	po::options_description desc("Allowed options");
	desc.add_options()
		("file",po::value<std::string>(),"path to yaml file")
		("camera",po::value<std::string>(),"camera name");
	po::variables_map vm;
	po::store(po::parse_command_line(argc,argv,desc),vm);
	po::notify(vm);
	if(vm.count("file")){
		file_name=vm["file"].as<std::string>();
	}
	else{
		std::cout<<"--file <path-to-yaml-file> [--camera <camera-name>]"<<std::endl;
		return 1;
	}
	if(vm.count("camera")){
		camera_name=vm["camera"].as<std::string>();
	}
	else{
		camera_name="/calibrated_camera";
	}
	ros::init(argc,argv,"camera_info_publisher");
	cameraInfoPublisher camera_info_publisher(file_name,camera_name);
	ros::spin();
	return 0;
}