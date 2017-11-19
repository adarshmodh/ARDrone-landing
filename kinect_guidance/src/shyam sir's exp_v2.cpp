#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <camera_calibration_parsers/parse.h>
#include <image_transport/camera_subscriber.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "ardrone_autonomy/Navdata.h"    //Accessing ardrone's published data
#include "sensor_msgs/Imu.h"
#include "ardrone_test/Drone_odo.h"

#include <stdio.h>
#include <math.h>
#include <termios.h>
#include <sstream>
#include <iostream>
#include <fstream>

//std_msgs::Float64 val;
ardrone_test::Drone_odo val;
ardrone_autonomy::Navdata drone_navdata;  	// Using this class variable for storing navdata

double t0;
ros::Publisher Num_pub;
ros::Subscriber pose_subscriber;		// ardrone navdata subsciber
image_transport::Publisher image_pub_;

void callFilter(const cv::Mat& inputImage, cv::Mat& outputImage);

void getCannyImage(const cv::Mat& inputImage, cv::Mat& outputImage);
void getThresholdImage(const cv::Mat& inputImage, cv::Mat& outputImage);
void getRedCircles(const cv::Mat& inputImage, cv::Mat& outputImage);

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

			cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

	}
  	catch (cv_bridge::Exception& e)
     	{
  			ROS_ERROR("cv_bridge exception: %s", e.what());
  		    return;
     	}


  	cv::imshow("view", cv_ptr->image);
  	cv::imwrite(file,cv_ptr->image);
  	cv::Mat outputImage;
  	callFilter(cv_ptr->image, outputImage);

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
     Num_pub = nh.advertise<ardrone_test::Drone_odo>("chatter1", 1000);

     image_transport::ImageTransport it(nh);
     //image_transport::CameraSubscriber sub = it.subscribe("/kinect2/qhd/image_color_rect", 1, imageCallback);
     Callbacks callbacks;
     // Useful when CameraInfo is being published
    //image_transport::CameraSubscriber sub_image_and_camera = it.subscribeCamera("/kinect2/qhd/image_color_rect", 1,&Callbacks::imageCallback,&callbacks);
     image_transport::CameraSubscriber sub_image_and_camera = it.subscribeCamera("/ardrone/image_raw", 1,&Callbacks::imageCallback,&callbacks);
     ros::spin();
     cv::destroyWindow("view");
}

void callFilter(const cv::Mat& inputImage, cv::Mat& outputImage)
{
   //getCannyImage(inputImage, outputImage);
	//getThresholdImage(inputImage, outputImage);
	getRedCircles(inputImage, outputImage);

}

void getCannyImage(const cv::Mat& inputImage, cv::Mat& outputImage)
{
     // Get a gray image - quite a bit of vision processing is done on grayscale images
     cv::Mat grayImage;
     cv::cvtColor(inputImage, grayImage, CV_RGB2GRAY);
     // Get an edge image - here we use the canny edge detection algorithm to get the edges
     double threshold1 = 20;
     double threshold2 = 50;
     int apertureSize = 3;
     // The smallest of threshold1 and threshold2 is used for edge linking,
     // the largest - to find initial segments of strong edges.  Play around
     // with these numbers to get desired result, and/or pre-process the
     // image, e.g. clean up, sharpen, blur).
     cv::Canny(grayImage, outputImage, threshold1, threshold2, apertureSize);
   }

void getThresholdImage(const cv::Mat& inputImage, cv::Mat& outputImage)
{
  // Get a gray image - quite a bit of vision processing is done on grayscale images
  cv::Mat grayImage;
  cv::cvtColor(inputImage, grayImage, CV_BGR2GRAY);
  int thresh = 128;
  double maxVal = 255;
  int thresholdType = CV_THRESH_BINARY;
  cv::threshold(grayImage, outputImage, thresh, maxVal, thresholdType);
}
void getRedCircles(const cv::Mat& inputImage, cv::Mat& outputImage)
{
	cv::Mat img_hsv;
	int iLastX = -1;
	int iLastY = -1;

	//Create a black image with the size as the camera output
	 cv::Mat imgLines = cv::Mat::zeros( img_hsv.size(), CV_8UC3 );


	cvtColor(inputImage,img_hsv,CV_BGR2HSV);
    cv::Mat img_hsv_upper;
    cv::Mat img_hsv_lower;

	cv::inRange(img_hsv,cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), img_hsv_lower);
	cv::inRange(img_hsv,cv::Scalar(160, 100, 100), cv::Scalar(179, 255, 255), img_hsv_upper);

	cv::Mat img_hsv_red;
	cv::addWeighted(img_hsv_lower, 1.0, img_hsv_upper, 1.0, 0.0, img_hsv_red);

	cv::GaussianBlur(img_hsv_red, img_hsv_red, cv::Size(9,9), 2,2);

	cv::Mat imgThresholded = img_hsv_red;
    //cv::Mat img_hsv_lower;

	//morphological opening (removes small objects from the foreground)
	cv::erode(imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5))  );
	cv::dilate( imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5))  );

	//morphological closing (removes small holes from the foreground)
	cv::dilate( imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5))  );
	cv::erode(imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5))  );

	 cv::Moments oMoments = cv::moments(imgThresholded);

	  double dM01 = oMoments.m01;
	  double dM10 = oMoments.m10;
	  double dArea = oMoments.m00;

	  // if the area <= 10000, I consider that the there are no object in the image and it's because of the noise, the area is not zero
	  	  if (dArea > 10000)
	  	  {
	  	   //calculate the position of the ball
	  	   int posX = dM10 / dArea;
	  	   int posY = dM01 / dArea;

	  	   if (iLastX >= 0 && iLastY >= 0 && posX >= 0 && posY >= 0)
	  	   {
	  	    //Draw a red line from the previous point to the current point
	  	    cv::line(imgLines, cv::Point(posX, posY), cv::Point(iLastX, iLastY), cv::Scalar(0,0,255), 2);
	  	   }

		   iLastX = posX;
		   iLastY = posY;
		  }



		  imshow("Thresholded Image", imgThresholded); //show the thresholded image

		  img_hsv = img_hsv + imgLines;
		  //imshow("Original", img_hsv); //show the original image

		ROS_INFO("got the image = ");


	   	ROS_INFO("%d, %d", iLastX, iLastY);

	outputImage = img_hsv;
  	 val.x = iLastY;
  	 val.y = iLastX;
//  	 ROS_INFO("%f", val.data);
  	 Num_pub.publish(val);

/*
  	       std::stringstream st1;
  	       st1<<iLastX;
  	       std::string ts1 = st1.str();
  	       cv::putText(img, ts1, cv::Point(0,20), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(255,0,0), 2.0);

  	       std::stringstream st2;
  	       st2<<iLastY;
  	       std::string ts2 = st2.str();
  	       cv::putText(img, ts2, cv::Point(0,40), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(255,0,0), 2.0);

  	      cv::imshow("OP", cv_ptr->image);
  	    cv::waitKey(3);
  	  image_pub_.publish(cv_ptr->toImageMsg());
*/
}


