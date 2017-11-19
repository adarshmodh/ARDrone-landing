#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

cv::Mat img;
cv_bridge::CvImagePtr cv_ptr;


void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	try
	{	
//			cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::16UC1);
			cv_ptr = cv_bridge::toCvCopy(msg, "16UC1");

	}
  	catch (cv_bridge::Exception& e)
     	{
  			ROS_ERROR("cv_bridge exception: %s", e.what());
  		    return;
     	}


  	cv::imshow("view", cv_ptr->image);
  	cv::waitKey(3);

  	float d_val = (float) cv_ptr->image.at<uint16_t>(msg->height/2,msg->width/2);   
  	ROS_INFO("depth = %f",d_val/1000 );
}
   
int main(int argc, char **argv)
{
     ros::init(argc, argv, "image_listener");
     ros::NodeHandle nh;
     cv::namedWindow("view");
     cv::startWindowThread();
     image_transport::ImageTransport it(nh);
     image_transport::Subscriber sub = it.subscribe("/kinect2/hd/image_depth_rect", 1, imageCallback);
     ros::spin();
     cv::destroyWindow("view");
}
