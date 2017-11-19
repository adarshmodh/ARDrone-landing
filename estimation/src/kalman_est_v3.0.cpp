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

float angx,angy,angz, est_x, est_y, est_z, pos_x, pos_y,pos_z;
int redX, redY;

double lat, lon, ele, ang_time,dt, tp;


void NumCallback(const ardrone_test::Drone_odo::ConstPtr& val);
void EstCallback(const ardrone_test::est_co::ConstPtr& est);
void poseCallback(const ardrone_autonomy::Navdata::ConstPtr & pose_message);		// drone actual data
void gpsCallback(const ardrone_autonomy::navdata_gps::ConstPtr & gps_message);		// drone actual data
void imuCallback(const sensor_msgs::Imu::ConstPtr & imu_message);		// drone actual data

using namespace cv;
using namespace std;

//------------------------------------------------ convenience method for
//                                                 using kalman filter with
//                                                 Point objects
cv::KalmanFilter KF;
cv::Mat s, p;
cv::Mat prediction, estimated;
cv::Mat_<float> measurement(3,1);
cv::Mat_<float> control(3,1);
Mat_<float> state(3, 1); // (x, y, Vx, Vy)

void initKalman()
{
    // Instantate Kalman Filter with
    // 3 dynamic parameters and 3 measurement parameters,
    // where my measurement is: 3D location of object,
    // and dynamic is: 3D location
    KF.init(3, 3, 3);

    measurement = Mat_<float>::zeros(3,1);
    measurement.at<float>(0, 0) = est_x;
    measurement.at<float>(1, 0) = est_y;
    measurement.at<float>(2, 0) = est_z;

    control = Mat_<float>::zeros(3,1);
    control.at<float>(0, 0) = drone_navdata.vx;
    control.at<float>(1, 0) = drone_navdata.vy;
    control.at<float>(2, 0) = 0;

    KF.statePre.setTo(0);
    KF.statePre.at<float>(0, 0) = pos_x;
    KF.statePre.at<float>(1, 0) = pos_y;
    KF.statePre.at<float>(2, 0) = pos_z;

    KF.statePost.setTo(0);
    KF.statePost.at<float>(0, 0) = pos_x;
    KF.statePost.at<float>(1, 0) = pos_y;
    KF.statePost.at<float>(2, 0) = pos_z;

    //setIdentity(KF.transitionMatrix);
    KF.transitionMatrix = *(cv::Mat_<float>(3, 3) << 1,0,0,	 0,1,0,  0,0,1);
    setIdentity(KF.measurementMatrix);
    setIdentity(KF.processNoiseCov, Scalar::all(1e-3)); //adjust this for faster convergence - but higher noise
    setIdentity(KF.measurementNoiseCov, Scalar::all(1e-1));
    setIdentity(KF.errorCovPost, Scalar::all(.1));
}

Mat kalmanPredict(float x, float y, float z)
{
	control(0) = x;
	control(1) = y;
	control(2) = 0;
    prediction = KF.predict(control);
    cout<<"predict : "<<control(0)<<" "<<control(1)<<endl;
    return prediction;
}

Mat kalmanCorrect(float x, float y, float z)
{
    measurement(0) = x;
    measurement(1) = y;
    measurement(1) = z;
    estimated = KF.correct(measurement);
//    cout<<"correct"<<endl;
    return estimated;
}


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

		int i=0;

	    initKalman();
		ofstream myfile;
		myfile.open("/home/icgel/kalman.txt", ios::out | ios::app );

while(1)
{
	myfile<<endl<<i<<setw(20);

	cout<<"Predict"<<drone_navdata.vx<<setw(20)<<drone_navdata.vy<<endl;

	p = kalmanPredict(drone_navdata.vx,drone_navdata.vy,0);
		//pos_x = p.at<float>(0);
	    //pos_y = p.at<float>(1);
	    //pos_z = p.at<float>(2);
	cout << "kalman prediction: " << pos_x << " " << pos_y << endl;

	//cout << "Correct " << est_x << setw(20) << est_y << endl;
	s = kalmanCorrect(est_x, est_y, est_z);
	    //pos_x = s.at<float>(0);
	    //pos_y = s.at<float>(1);
	    //pos_z = s.at<float>(2);
	//cout << "kalman corrected state: " << pos_x << " " << pos_y << endl;
	    i =i+1;
	ros::spinOnce();
}
myfile.close();
ros::spin();
}
void poseCallback(const ardrone_autonomy::Navdata::ConstPtr & pose_message )
{
	drone_navdata.vx	= 	pose_message->vx*0.001;
	drone_navdata.vy 	= 	pose_message->vy*0.001;
	drone_navdata.vz 	= 	pose_message->vz*0.001;
	drone_navdata.ax	= 	pose_message->ax;
	drone_navdata.ay 	= 	pose_message->ay;
	drone_navdata.az 	= 	pose_message->az;
	drone_navdata.rotX	= 	pose_message->rotX;
	drone_navdata.rotY	= 	pose_message->rotY;
	drone_navdata.rotZ	= 	pose_message->rotZ;
	drone_navdata.magX	= 	pose_message->magX;
	drone_navdata.magY	= 	pose_message->magY;
	drone_navdata.magZ	= 	pose_message->magZ;
	drone_navdata.altd 	= 	pose_message->altd*0.001;
	dt = (pose_message->tm-tp)*0.000001;
	drone_navdata.tm	= 	pose_message->tm;
	drone_navdata.header	= 	pose_message->header;
	tp = drone_navdata.tm;
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



