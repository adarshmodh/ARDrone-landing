#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <camera_calibration_parsers/parse.h>
#include <image_transport/camera_subscriber.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "ardrone_autonomy/Navdata.h"    //Accessing ardrone's published data
#include "sensor_msgs/Imu.h"

#include <stdio.h>
#include <math.h>
#include <termios.h>
#include <sstream>
#include <iostream>
#include <fstream>


cv::Mat img;
cv_bridge::CvImagePtr cv_ptr;
int count=1;

class Callbacks
{
public:
Callbacks() : is_first_image_(true), has_camera_info_(false), count_(0){
 }
void imageCallback(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& info)
{
	std::string filename;
	// save the CameraInfo

	std::ofstream myfile;
	myfile.open("/home/icgel/video_test.txt", std::ios::out | std::ios::app );
	myfile<<info->header.seq<<std::setw(25);
	myfile<<info->header.stamp<<std::endl;

   std::string file;

   std::stringstream ss1;
   ss1<<info->header.seq;
   std::string s1 = ss1.str();
   std::string ext = ".jpg";

   file = s1 + ext;


   try
	{

			cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC3);

	}
  	catch (cv_bridge::Exception& e)
     	{
  			ROS_ERROR("cv_bridge exception: %s", e.what());
  		    return;
     	}


  	cv::imshow("view", cv_ptr->image);
  	cv::imwrite(file,cv_ptr->image);
  	//count++;
  	cv::waitKey(3);
	myfile.close();
}
private:
bool is_first_image_;
bool has_camera_info_;
size_t count_;
};
int main(int argc, char **argv)
{
     ros::init(argc, argv, "image_listener");
     ros::NodeHandle nh;
     cv::namedWindow("view");
     cv::startWindowThread();
     image_transport::ImageTransport it(nh);
     //image_transport::CameraSubscriber sub = it.subscribe("/kinect2/qhd/image_color_rect", 1, imageCallback);
     Callbacks callbacks;
     // Useful when CameraInfo is being published
    //image_transport::CameraSubscriber sub_image_and_camera = it.subscribeCamera("/kinect2/qhd/image_color_rect", 1,&Callbacks::imageCallback,&callbacks);
     image_transport::CameraSubscriber sub_image_and_camera = it.subscribeCamera("/ardrone/image_raw", 1,&Callbacks::imageCallback,&callbacks);
     ros::spin();
     cv::destroyWindow("view");
}

