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


#define drawCross( center, color, d )                                 \
cv::line( img, Point( center.x - d, center.y - d ), Point( center.x + d, center.y + d ), color, 2, CV_AA, 0); \
cv::line( img, Point( center.x + d, center.y - d ), Point( center.x - d, center.y + d ), color, 2, CV_AA, 0 )


using namespace cv;
using namespace std;

int main( int argc, char **argv)
{
	ros::init(argc, argv, "mousefollower");
	ros::NodeHandle nh;

cv::KalmanFilter KF(4, 2, 0);
//cv::Point mousePos;
//GetCursorPos(&mousePos);

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


while(1)
{
 // First predict, to update the internal statePre variable
 cv::Mat prediction = KF.predict();
 cv::Point predictPt(prediction.at<float>(0),prediction.at<float>(1));

 // Get mouse point
 //GetCursorPos(&mousePos);
 measurement(0) = 200;
 measurement(1) = 100;

 // The update phase
 cv::Mat estimated = KF.correct(measurement);

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

 cv::waitKey(10);
}

ros::spin();

}

