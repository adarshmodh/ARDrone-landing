#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
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
#include "geometry_msgs/Twist.h"			//for command velocity
#include "geometry_msgs/Vector3.h"			//for command velocity
#include "std_msgs/Empty.h"	 				//For take off and landing
#include "ardrone_autonomy/CamSelect.h"    	// For toggling camera
#include "ardrone_autonomy/Navdata.h"    //Accessing ardrone's published data
#include "sensor_msgs/Imu.h"
#include "ardrone_autonomy/navdata_gps.h"    //Accessing ardrone's published data
#include <termios.h>
#include <sstream>
#include <fstream>
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "ardrone_test/Drone_odo.h"
#include "ardrone_test/est_co.h"

// Variables for Subscribing
ros::Subscriber pose_subscriber;		// ardrone navdata subsciber
ros::Subscriber gps_subscriber;		// ardrone navdata subsciber
ros::Subscriber imu_subscriber;		// ardrone navdata subsciber
ros::Subscriber est_sub;		// ardrone navdata subsciber
ros::Subscriber num_sub;

ros::Publisher kalman_est_pub;
ardrone_test::est_co estdata;

ardrone_autonomy::Navdata drone_navdata;  	// Using this class variable for storing navdata
sensor_msgs::Imu ang;

float angx,angy,angz, est_x, est_y, est_z;
int redX, redY;

double lat, lon, ele, ang_time;


void NumCallback(const ardrone_test::Drone_odo::ConstPtr& val);
void EstCallback(const ardrone_test::est_co::ConstPtr& est);
void poseCallback(const ardrone_autonomy::Navdata::ConstPtr & pose_message);		// drone actual data
void gpsCallback(const ardrone_autonomy::navdata_gps::ConstPtr & gps_message);		// drone actual data
void imuCallback(const sensor_msgs::Imu::ConstPtr & imu_message);		// drone actual data

#define drawCross( center, color, d )                                 \
cv::line( img, Point( center.x - d, center.y - d ), Point( center.x + d, center.y + d ), color, 2, CV_AA, 0); \
cv::line( img, Point( center.x + d, center.y - d ), Point( center.x - d, center.y + d ), color, 2, CV_AA, 0 )


using namespace cv;
using namespace std;

int main( int argc, char **argv)
{
	ros::init(argc, argv, "kalman_estimation");
	ros::NodeHandle ni;

	pose_subscriber = ni.subscribe("/ardrone/navdata", 10, poseCallback);
	imu_subscriber = ni.subscribe("/ardrone/imu", 10, imuCallback);
	gps_subscriber = ni.subscribe("/ardrone/navdata_gps", 10, gpsCallback);
	est_sub = ni.subscribe("Coordinate", 100, EstCallback);
	num_sub = ni.subscribe("/chatter1", 100, NumCallback);

	kalman_est_pub = ni.advertise<ardrone_test::Drone_odo>("estimation_data", 1000);

cv::KalmanFilter KF(4, 2, 0);
//cv::Point mousePos;
//GetCursorPos(&mousePos)

// intialization of KF...
KF.transitionMatrix = *(cv::Mat_<float>(4, 4) << 1,0,1,0,   0,1,0,1,  0,0,1,0,  0,0,0,1);
cv::Mat_<float> measurement(2,1); measurement.setTo(Scalar(0));

KF.statePre.at<float>(0) = 200;
KF.statePre.at<float>(1) = 100;
KF.statePre.at<float>(2) = 0;
KF.statePre.at<float>(3) = 0;
cv::setIdentity(KF.measurementMatrix);
cv::setIdentity(KF.processNoiseCov, Scalar::all(1e-4));
cv::setIdentity(KF.measurementNoiseCov, Scalar::all(10));
cv::setIdentity(KF.errorCovPost, Scalar::all(.1));
// Image to show mouse tracking

cv::Mat img(600, 800, CV_8UC3);
cv::vector<Point> mousev,kalmanv;
mousev.clear();
kalmanv.clear();

while(ros::ok())
{
 // First predict, to update the internal statePre variable
 cv::Mat prediction = KF.predict();
 cv::Point predictPt(prediction.at<float>(0),prediction.at<float>(1));

 // Get mouse point
 //GetCursorPos(&mousePos);
 measurement(0) = 400;
 measurement(1) = 200;

 // The update phase
 cv::Mat estimated = KF.correct(measurement);
/*
 cv::Point statePt(estimated.at<float>(0),estimated.at<float>(1));
 cv::Point measPt(measurement(0),measurement(1));

 	 // plot points
    cv::imshow("mouse kalman", img);
    img = Scalar::all(0);

    mousev.push_back(measPt);
    kalmanv.push_back(statePt);
    drawCross( statePt, Scalar(255,255,255), 5 );
    drawCross( measPt, Scalar(0,0,255), 5 );

    for (int i = 0; i < mousev.size()-1; i++)
     line(img, mousev[i], mousev[i+1], Scalar(255,255,0), 1);

    for (int i = 0; i < kalmanv.size()-1; i++)
     line(img, kalmanv[i], kalmanv[i+1], Scalar(0,155,255), 1);
*/

    estdata.x = estimated.at<float>(0);
    estdata.y = prediction.at<float>(0);
    estdata.z = 0;
    kalman_est_pub.publish(estdata);

 cv::waitKey(10);

}

ros::spin();
// cv::destroyWindow("mouse kalman");
return 0;
}
void poseCallback(const ardrone_autonomy::Navdata::ConstPtr & pose_message )
{
	drone_navdata.vx	= 	pose_message->vx;
	drone_navdata.vy 	= 	pose_message->vy;
	drone_navdata.vz 	= 	pose_message->vz;
	drone_navdata.ax	= 	pose_message->ax;
	drone_navdata.ay 	= 	pose_message->ay;
	drone_navdata.az 	= 	pose_message->az;
	drone_navdata.rotX	= 	pose_message->rotX;
	drone_navdata.rotY	= 	pose_message->rotY;
	drone_navdata.rotZ	= 	pose_message->rotZ;
	drone_navdata.magX	= 	pose_message->magX;
	drone_navdata.magY	= 	pose_message->magY;
	drone_navdata.magZ	= 	pose_message->magZ;
	drone_navdata.altd 	= 	pose_message->altd;
	drone_navdata.tm	= 	pose_message->tm;
	drone_navdata.header	= 	pose_message->header;
}
void imuCallback(const sensor_msgs::Imu::ConstPtr & imu_message)
{
	ang.header	= 	imu_message->header;
	ang.angular_velocity	= 	imu_message->angular_velocity;
	}

void gpsCallback(const ardrone_autonomy::navdata_gps::ConstPtr & gps_message)
{
	lat		= 	gps_message->latitude;
	lon 	= 	gps_message->longitude;
	ele 	= 	gps_message->elevation;
}

void NumCallback(const ardrone_test::Drone_odo::ConstPtr& val)
{
	//cout<<"TaRgOt:"<< val->x<< val->y;
	redX = val->x;
	redY = val->y;
}

void EstCallback(const ardrone_test::est_co::ConstPtr& est)
{
  //cout<<"Target:"<< val->x<< val->y;
	est_x = est->x;
	est_y = -est->y;
	est_z = est->z-1;
	//cout<<"Target:"<< est_x<< setw(25)<<est_y<<setw(25) << est_z;
}



