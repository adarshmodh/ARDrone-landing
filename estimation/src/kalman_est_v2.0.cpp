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

using namespace cv;
using namespace std;

//------------------------------------------------ convenience method for
//                                                 using kalman filter with
//                                                 Point objects
cv::KalmanFilter KF;
cv::Mat_<float> measurement(2,1);
Mat_<float> state(4, 1); // (x, y, Vx, Vy)

void initKalman(float x, float y)
{
    // Instantate Kalman Filter with
    // 4 dynamic parameters and 2 measurement parameters,
    // where my measurement is: 2D location of object,
    // and dynamic is: 2D location and 2D velocity.
    KF.init(4, 2, 0);

    measurement = Mat_<float>::zeros(2,1);
    measurement.at<float>(0, 0) = x;
    measurement.at<float>(0, 0) = y;


    KF.statePre.setTo(0);
    KF.statePre.at<float>(0, 0) = x;
    KF.statePre.at<float>(1, 0) = y;

    KF.statePost.setTo(0);
    KF.statePost.at<float>(0, 0) = x;
    KF.statePost.at<float>(1, 0) = y;

    //setIdentity(KF.transitionMatrix);
    KF.transitionMatrix = *(cv::Mat_<float>(4, 4) << 1,0,1,0, 0,1,0,1,   0,0,1,0,	0,0,0,1);
    setIdentity(KF.measurementMatrix);
    setIdentity(KF.processNoiseCov, Scalar::all(.005)); //adjust this for faster convergence - but higher noise
    setIdentity(KF.measurementNoiseCov, Scalar::all(1e-1));
    setIdentity(KF.errorCovPost, Scalar::all(.1));
}

Point kalmanPredict()
{
    Mat prediction = KF.predict();
    Point predictPt(prediction.at<float>(0),prediction.at<float>(1));
    return predictPt;
}

Point kalmanCorrect(float x, float y)
{
    measurement(0) = x;
    measurement(1) = y;
    Mat estimated = KF.correct(measurement);
    Point statePt(estimated.at<float>(0),estimated.at<float>(1));
    return statePt;
}

//------------------------------------------------ main



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

	Point s, p;

	    initKalman(0, 0);

	    p = kalmanPredict();
	    cout << "kalman prediction: " << p.x << " " << p.y << endl;
	    /*
	     * output is: kalman prediction: 0 0
	     *
	     * note 1:
	     *         ok, the initial value, not yet new observations
	     */

	    s = kalmanCorrect(10, 10);
	    cout << "kalman corrected state: " << s.x << " " << s.y << endl;
	    /*
	     * output is: kalman corrected state: 5 5
	     *
	     * note 2:
	     *         ok, kalman filter is smoothing the noisy observation and
	     *         slowly "following the point"
	     *         .. how faster the kalman filter follow the point is
	     *            processNoiseCov parameter
	     */

	    p = kalmanPredict();
	    cout << "kalman prediction: " << p.x << " " << p.y << endl;
	    /*
	     * output is: kalman prediction: 5 5
	     *
	     * note 3:
	     *         mhmmm, same as the last correction, probabilly there are so few data that
	     *         the filter is not predicting anything..
	     */

	    s = kalmanCorrect(20, 20);
	    cout << "kalman corrected state: " << s.x << " " << s.y << endl;
	    /*
	     * output is: kalman corrected state: 10 10
	     *
	     * note 3:
	     *         ok, same as note 2
	     */

	    p = kalmanPredict();
	    cout << "kalman prediction: " << p.x << " " << p.y << endl;
	    s = kalmanCorrect(30, 30);
	    cout << "kalman corrected state: " << s.x << " " << s.y << endl;
	    /*
	     * output is: kalman prediction: 10 10
	     *            kalman corrected state: 16 16
	     *
	     * note 4:
	     *         ok, same as note 2 and 3
	     */


	    /*
	     * now let's say I don't received observation for few frames,
	     * I want anyway to update the kalman filter to predict
	     * the future states of my system
	     *
	     */
	    for(int i=0; i<5; i++) {
	        p = kalmanPredict();
	        cout << "kalman prediction: " << p.x << " " << p.y << endl;
	    }
	    /*
	     * output is: kalman prediction: 16 16
	     * kalman prediction: 16 16
	     * kalman prediction: 16 16
	     * kalman prediction: 16 16
	     * kalman prediction: 16 16
	     *
	     * !!! kalman filter is still on 16, 16..
	     *     no future prediction here..
	     *     I'm exprecting the point to go further..
	     *     why???
	     *
	     */

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



