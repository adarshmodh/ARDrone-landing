#include "ros/ros.h"

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include "ardrone_autonomy/Navdata.h"    //Accessing ardrone's published data
#include "ardrone_test/Drone_odo.h"

#include <opencv2/video/tracking.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "opencv/cv.hpp"
#include "opencv2/core/version.hpp"
#include <stdio.h>
#include <math.h>
#include <iostream>
#include <vector>

#if CV_MAJOR_VERSION ==2
#elif CV_MAJOR_VERSION ==3
#endif


static const std::string OPENCV_WINDOW = "Image window";
cv::Mat img;
//std_msgs::Float64 val;
ardrone_test::Drone_odo val;
ardrone_autonomy::Navdata drone_navdata;  	// Using this class variable for storing navdata

double t0;
ros::Publisher Num_pub;
ros::Subscriber pose_subscriber;		// ardrone navdata subsciber
image_transport::Publisher image_pub_;
cv_bridge::CvImagePtr cv_ptr;
int count = 0;
void poseCallback(const ardrone_autonomy::Navdata::ConstPtr & pose_message);		// drone actual data

void imag_test(cv::Mat img);
void callFilter(const cv::Mat& inputImage, cv::Mat& outputImage);

void getCannyImage(const cv::Mat& inputImage, cv::Mat& outputImage);
void getThresholdImage(const cv::Mat& inputImage, cv::Mat& outputImage);
void getRedCircles(const cv::Mat& inputImage, cv::Mat& outputImage);
void imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
		double t1 = ros::Time::now().toSec(); //ros has a 'Time' function in which the current time


       try
       {
         cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

       }
       catch (cv_bridge::Exception& e)
       {
         ROS_ERROR("cv_bridge exception: %s", e.what());
         return;
       }

       img = cv_ptr->image;


       cv::Mat outputImage;
       callFilter(cv_ptr->image, outputImage);
//       cv::imshow("Output", outputImage);
/*
       std::stringstream ss1;
       ss1<<drone_navdata.tm;
       std::string s1 = ss1.str();
       cv::putText(img, s1, cv::Point(0,20), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(255,0,0), 2.0);



       std::stringstream ss1;
       ss1<<drone_navdata.ax;
       std::string s1 = ss1.str();
       cv::putText(img, s1, cv::Point(0,20), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(255,0,0), 2.0);

       std::stringstream ss2;
       ss2<<drone_navdata.ay;
       std::string s2 = ss2.str();
       cv::putText(img, s2, cv::Point(0,40), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(255,0,0), 2.0);

       std::stringstream ss3;
       ss3<<drone_navdata.az;
       std::string s3 = ss3.str();
       cv::putText(img, s3, cv::Point(0,60), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(255,0,0), 2.0);

       std::stringstream ss4;
       ss4<<drone_navdata.rotX;
       std::string s4 = ss4.str();
       cv::putText(img, s4, cv::Point(0,80), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(255,0,0), 2.0);

       std::stringstream ss5;
       ss5<<drone_navdata.rotY;
       std::string s5 = ss5.str();
       cv::putText(img, s5, cv::Point(0,100), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(255,0,0), 2.0);

       std::stringstream ss6;
       ss6<<drone_navdata.rotZ;
       std::string s6 = ss6.str();
       cv::putText(img, s6, cv::Point(0,120), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(255,0,0), 2.0);
*/
       double t = t1 - t0;
       std::stringstream ss7;
       ss7<<t;
       std::string s7 = ss7.str();
       cv::putText(img, s7, cv::Point(0,140), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(255,0,0), 2.0);


//       	 cv::imshow(OPENCV_WINDOW, img);
       	 cv::waitKey(3);

       	imag_test(img);

       	image_pub_.publish(cv_ptr->toImageMsg());

//       ROS_INFO("got the image");
       // Update GUI Window

     }
   int main(int argc, char** argv)
   {
     ros::init(argc, argv, "image_converter");

     ros::NodeHandle ng_;
     pose_subscriber = ng_.subscribe("/ardrone/navdata", 10, poseCallback);
     image_transport::ImageTransport it_(ng_);
     image_transport::Subscriber image_sub_;

     t0 = ros::Time::now().toSec(); //ros has a 'Time' function in which the current time

  //   cv::namedWindow(OPENCV_WINDOW);
     cv::startWindowThread();
          // Subscrive to input video feed and publish output video feed
          image_sub_ = it_.subscribe("/ardrone/image_raw", 10, imageCb);
          image_pub_ = it_.advertise("/Opencv_img/image_raw", 10);

          //Num_pub = ng_.advertise<std_msgs::Float64>("chatter1", 1000);

          Num_pub = ng_.advertise<ardrone_test::Drone_odo>("chatter1", 1000);

  //    imag_test(img);

//	cv::destroyWindow(OPENCV_WINDOW);

     ros::spin();
     return 0;
   }
void poseCallback(const ardrone_autonomy::Navdata::ConstPtr & pose_message )
{
	drone_navdata.ax	= 	pose_message->ax;
	drone_navdata.ay	= 	pose_message->ay;
	drone_navdata.az	= 	pose_message->az;
   	drone_navdata.rotX	= 	pose_message->rotX;
   	drone_navdata.rotY	= 	pose_message->rotY;
   	drone_navdata.rotZ	= 	pose_message->rotZ;
   	drone_navdata.altd 	= 	pose_message->altd;
   	drone_navdata.tm 	= 	pose_message->tm;
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
//	imshow("HSV", img_hsv); //show the img_hsv_lower
    cv::Mat img_hsv_upper;
    cv::Mat img_hsv_lower;

    cv::inRange(img_hsv,cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), img_hsv_lower);
//    imshow("HSV LOWER", img_hsv_lower); //show the img_hsv_lower

    cv::inRange(img_hsv,cv::Scalar(160, 100, 100), cv::Scalar(179, 255, 255), img_hsv_upper);
//    imshow("hsv UPPER", img_hsv_upper); //img_hsv_upper

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



//		  imshow("Thresholded Image", imgThresholded); //show the thresholded image

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


void imag_test(cv::Mat img)
{
       	int width = img.cols;
     	int height = img.rows;
		//ROS_INFO("got the image = ");


	   	//ROS_INFO("%d, %d", width, height);



/*
       	 if(count<=100 && count>=50)
       	 {
           	 val.data = count;
           	 ROS_INFO("%f", val.data);
           	 Num_pub.publish(val);

       	 }
*/
       count++;
}
