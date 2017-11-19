#include "ros/ros.h"

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include "ardrone_autonomy/Navdata.h"    //Accessing ardrone's published data
#include "ardrone_test/Drone_odo.h"
#include "ardrone_test/est_co.h"

#include <opencv2/video/tracking.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "opencv/cv.hpp"
#include "opencv2/core/version.hpp"
#include <stdio.h>
#include <math.h>
#include <iostream>
#include <vector>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>



ros::Publisher Num_pub;
ros::Publisher co_pub;
ardrone_test::Drone_odo val;
ardrone_test::est_co value;


//#define EXACT
#define APPROXIMATE


#ifdef EXACT
#include <message_filters/sync_policies/exact_time.h>
#endif
#ifdef APPROXIMATE
#include <message_filters/sync_policies/approximate_time.h>
#endif

#define deg2rad 0.0174533
#define order 20                                             ////change order for median filter
#define samplerate 1											////downsampling rate

using namespace std;
//using namespace sensor_msgs;
using namespace message_filters;

cv::Mat color_img,depth_img;
cv::Mat imgThresholded,img_hsv_upper,img_hsv_lower;
cv_bridge::CvImagePtr img_ptr_color;
cv_bridge::CvImagePtr img_ptr_depth;
ardrone_autonomy::Navdata drone_navdata;  	// Using this class variable for storing navdata
image_transport::Publisher image_pub_;

float x_m,y_m,z_m;
float depth_val=0,temp_depth=0,prev_depth=0;

float sampled_x_m=0,sampled_z_m=0,sampled_y_m=0;

float depth_array[order]={0},y_array[order]={0},z_array[order]={0},fltr_depth_val=0,fltr_y_val=0,fltr_z_val=0;

float prev_depth_val;
int flag1=0,i=0;
int sample=samplerate;

void getRedCircles(const cv::Mat& inputImage, cv::Mat& outputImage);
void callFilter(const cv::Mat& inputImage, cv::Mat& outputImage);

void med_filter_depth(int order_val)
{

	for (int c = 0 ; c < ( order_val - 1 ); c++)
	  {
	    for (int d = 0 ; d < (order_val - c - 1); d++)
	    {
	      if (depth_array[d] > depth_array[d+1]) /* For decreasing order use < */
	      {
	        float swap = depth_array[d];
	        depth_array[d]   = depth_array[d+1];
	        depth_array[d+1] = swap;
	      }
	    }
	  }

	fltr_depth_val = depth_array[order_val/2];
}

void med_filter_y(int order_val)
{

	for (int c = 0 ; c < ( order_val - 1 ); c++)
	  {
	    for (int d = 0 ; d < (order_val - c - 1); d++)
	    {
	      if (y_array[d] > y_array[d+1]) /* For decreasing order use < */
	      {
	        float swap = y_array[d];
	        y_array[d]   = y_array[d+1];
	        y_array[d+1] = swap;
	      }
	    }
	  }

	fltr_y_val = y_array[order_val/2];
}

void med_filter_z(int order_val)
{

	for (int c = 0 ; c < ( order_val - 1 ); c++)
	  {
	    for (int d = 0 ; d < (order_val - c - 1); d++)
	    {
	      if (z_array[d] > z_array[d+1]) /* For decreasing order use < */
	      {
	        float swap = z_array[d];
	        z_array[d]   = z_array[d+1];
	        z_array[d+1] = swap;
	      }
	    }
	  }

	fltr_z_val = z_array[order_val/2];
}

void callback( const sensor_msgs::ImageConstPtr& msg_rgb , const sensor_msgs::ImageConstPtr& msg_depth )
{

	try
	{
        img_ptr_depth = cv_bridge::toCvCopy(*msg_depth, sensor_msgs::image_encodings::TYPE_16UC1);
	}
	catch (cv_bridge::Exception& e)
	{
	        ROS_ERROR("cv_bridge exception:  %s", e.what());
	        return;
	}

	try
	{
        img_ptr_color = cv_bridge::toCvCopy(*msg_rgb, sensor_msgs::image_encodings::TYPE_8UC3);
	}
  	catch (cv_bridge::Exception& e)
    {
  			ROS_ERROR("cv_bridge exception: %s", e.what());
  		    return;
    }

  	color_img = img_ptr_color->image;

  	cv::Mat outputImage;
  	callFilter(color_img, outputImage);

  	//cv::imshow("view", color_img);

  	cv::waitKey(3);

 	depth_img = img_ptr_depth->image;

 	int depth_count=0,l=0,px=0,py=0,pix_count=0;


 	for(int b=0;b<img_hsv_upper.rows;b++)
 	 	{
 	 	  for (int a=0;a<img_hsv_upper.cols;a++)
 	 	    {
 	 		  uint pix_int =  (uint)img_hsv_upper.at<uchar>(b,a);

 	 		  static int flag2=0;

 	 		  if(pix_int>0)
 	 		   {
 	 			   temp_depth = (float) depth_img.at<uint16_t>(b, a);

 	 			   if(temp_depth>0 && temp_depth<6000)
 	 			  	{
 	 				   px = px + a ;
 	 				   py = py + b ;
 	 				   pix_count++;

 	 				   if(flag2==1)
 	 				   {
 	 					   if(abs(prev_depth-temp_depth)<150)
 	 					   {
 	 						   depth_val = depth_val + temp_depth;
 	 						   depth_count++;
 	 						   prev_depth = depth_val/depth_count;
 	 					   }
 	 				   }
 	 				   else
 	 					{
 	 					   prev_depth = temp_depth;
 	 					   flag2=1;
 	 					}


 	 			  	}

 	 	    }
 	 	}
 	 	}
 	if(pix_count==0)
 	{
 		px=py=0;
 	}
 	else
 	{
 		px = px/pix_count;
 		py = py/pix_count;
 	}


 	 if(depth_count==0)
 	 depth_val = 0;
 	 else
 	 depth_val = depth_val/depth_count;


 	if((px==0) && (py==0))
 	 {
 		ROS_INFO("helloo");
 		val.x = px;
 		val.y = py;
 		Num_pub.publish(val);
 		ROS_INFO("pixel co-ordinates = %d,%d",val.x,val.y);

 	 	//value.x=0;value.y=0;value.z=0;
 	 	co_pub.publish(value);
 	 	ROS_INFO("The real world co-ordinates of the centroid = %f, %f, %f",value.x,value.y,value.z );
 	 }
 	else if(depth_val>0)
 	 {
 		depth_val = depth_val/1000;

 		x_m = depth_val;
 		y_m = tan((px-512/2)*83*deg2rad/512)*(x_m);
 		z_m = tan((py-424/2)*53.8*deg2rad/424)*(x_m);

 		depth_array[i] = x_m;
 		y_array[i] = y_m;
 		z_array[i] = z_m;
 		i++;
 		if(i==order)
 			i=0;


 		if(flag1<=30)
 			flag1++;

 		if(flag1==31)
 		{

 			med_filter_depth(order);
 			med_filter_y(order);
 			med_filter_z(order);
 		}


 	sampled_x_m = sampled_x_m + fltr_depth_val;
 	sampled_y_m = sampled_y_m + fltr_y_val;
 	sampled_z_m = sampled_z_m + fltr_z_val;
 	sample--;

 	if(sample==0)
 	{
 		sampled_x_m = sampled_x_m/samplerate;
 		sampled_y_m = sampled_y_m/samplerate;
 		sampled_z_m = sampled_z_m/samplerate;

 		val.x = px;
 		val.y = py;
 		Num_pub.publish(val);
 		ROS_INFO("pixel co-ordinates = %d,%d",val.x,val.y);

 		sampled_x_m = (int) (sampled_x_m*100);
 		sampled_x_m = (float) (sampled_x_m/100);
 		sampled_y_m = (int) (sampled_y_m*100);
 		sampled_y_m = (float) (sampled_y_m/100);
 		sampled_z_m = (int) (sampled_z_m*100);
 		sampled_z_m = (float) (sampled_z_m/100);

 		 			value.x =  sampled_x_m;
 		 			value.y =  sampled_y_m;
 		 	 		value.z =  sampled_z_m;

 		 	 		co_pub.publish(value);
 		 	 		ROS_INFO("The real world co-ordinates of the centroid = %f, %f, %f",value.x,value.y,value.z );

 		 	 		sample=samplerate;sampled_x_m=0;sampled_y_m=0;sampled_z_m=0;

 	}
	}
 	else
 	{
 		Num_pub.publish(val);
 		co_pub.publish(value);
 		ROS_INFO("The real world co-ordinates of the centroid = %f, %f, %f",value.x,value.y,value.z );
 	}
}


int main(int argc, char **argv)
{
     ros::init(argc, argv, "kinect_guidance");
     ros::NodeHandle nh;
     cv::namedWindow("view");
     cv::startWindowThread();

   /*  image_transport::ImageTransport it(nh);
     image_transport::Subscriber sub_color = it.subscribe("/kinect2/qhd/image_color_rect", 1, imageCallback);
     image_transport::Subscriber sub_color = it.subscribe("/kinect2/qhd/image_depth_rect", 1, imageCallback);
    */

     Num_pub = nh.advertise<ardrone_test::Drone_odo>("chatter1", 1000);
     co_pub = nh.advertise<ardrone_test::est_co>("Coordinate", 1000);



     message_filters::Subscriber<sensor_msgs::Image> subscriber_depth( nh , "/kinect2/sd/image_depth_rect" , 1 );
     message_filters::Subscriber<sensor_msgs::Image> subscriber_rgb( nh , "/kinect2/sd/image_color_rect" , 1 );


   #ifdef EXACT
       typedef sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
   #endif
   #ifdef APPROXIMATE
       typedef sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
   #endif


     // ExactTime or ApproximateTime take a queue size as its constructor argument, hence MySyncPolicy(10)
     Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), subscriber_rgb, subscriber_depth );
     sync.registerCallback(boost::bind(&callback, _1, _2));


     ros::spin();


     cv::destroyWindow("view");
}


void callFilter(const cv::Mat& inputImage, cv::Mat& outputImage)
{

	getRedCircles(inputImage, outputImage);

}


void getRedCircles(const cv::Mat& inputImage, cv::Mat& outputImage)
{
	cv::Mat img_hsv;
//	int iLastX = -1;
//	int iLastY = -1;


	//Create a black image with the size as the camera output
	 cv::Mat imgLines = cv::Mat::zeros( img_hsv.size(), CV_8UC1 );


	cvtColor(inputImage,img_hsv,CV_BGR2HSV);

//	cv::inRange(img_hsv,cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), img_hsv_lower);
//	imshow("HSV LOWER", img_hsv_lower); //show the img_hsv_lower

	cv::inRange(img_hsv,cv::Scalar(160, 100, 100), cv::Scalar(179, 255, 255), img_hsv_upper);
//	imshow("HSV UPPER", img_hsv_upper); //img_hsv_upper

//	cv::Mat img_hsv_red;
	//cv::addWeighted(img_hsv_lower, 1.0, img_hsv_upper, 1.0, 0.0, img_hsv_red);

//	cv::GaussianBlur(img_hsv_upper, img_hsv_upper, cv::Size(9,9), 2,2);

	imgThresholded = img_hsv_upper;

	imshow("Thresholded Image", imgThresholded); //show the thresholded image

	//cv::Mat img_hsv_lower;

	//morphological opening (removes small objects from the foreground)
//	cv::erode(imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5))  );
//	cv::dilate( imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5))  );

	//morphological closing (removes small holes from the foreground)
//	cv::dilate( imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5))  );
//	cv::erode(imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5))  );


/*	 cv::Moments oMoments = cv::moments(imgThresholded);

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




		  img_hsv = img_hsv + imgLines;
		  //imshow("Original", img_hsv); //show the original image

		  outputImage = img_hsv;

/*		ROS_INFO("got the image = ");


	   	ROS_INFO("%d, %d", iLastX, iLastY);


  	 	 val.x = iLastY;
  	 	 val.y = iLastX;
//  	 ROS_INFO("%f", val.data);
  	 	 Num_pub.publish(val);
*/

}


