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
#include "std_msgs/String.h"


using namespace cv;

using namespace std;

cv::Point point=cvPoint(-1,-1);
cv::Point last_point;

cv::Vector<Point> mousev,kalmanv;

void my_mouse_callback(int event, int x, int y, int flags, void* param)
{
CV_EVENT_LBUTTONDBLCLK;
last_point=point;
point=cvPoint(x,y);

}

IplImage *img;

int main(int argc, char **argv)
{

ros::init(argc, argv, "mousefollower");
ros::NodeHandle nh;

IplImage* img = cvCreateImage( cvSize(500,500), 8, 3 );

CvKalman* kalman = cvCreateKalman( 4, 2, 0 );

CvMat* state = cvCreateMat( 4, 1, CV_32FC1 );

CvMat* processnoise=cvCreateMat(4,1,CV_32FC1);

CvMat* measurement =cvCreateMat(2,1,CV_32FC1);
cvZero(measurement);

char code = (char)-1;

cvNamedWindow( "kalman", 1 );

cvSetMouseCallback("kalman",my_mouse_callback,NULL);

while(1)
{
if(point.x<0 || point.y<0)
{
kalman->state_pre->data.fl[0]=point.x;
kalman->state_pre->data.fl[1]=point.y;
kalman->state_pre->data.fl[2]=0;
kalman->state_pre->data.fl[3]=0;
}
float trans_vals[] = {1,0,1,0,0,1,0,1,0,0,1,0,0,0,0,0};
/*CvMat tran;
cvInitMatHeader(&tran,4,4,CV_32FC1,trans_vals);
kalman->transition_matrix = cvCloneMat(&tran);*/

CvMat* tran=new CvMat;
cvInitMatHeader(tran,4,4,CV_32FC1,trans_vals);
//memcpy(static_cast(kalman->transition_matrix->data.fl),static_cast(tran),sizeof(tran));

cvSetIdentity( kalman->measurement_matrix);
cvSetIdentity( kalman->process_noise_cov, cvRealScalar(1e-4) );
cvSetIdentity( kalman->measurement_noise_cov, cvRealScalar(1e-1) );
cvSetIdentity( kalman->error_cov_post, cvRealScalar(.1) );

mousev.clear();
kalmanv.clear();

while(1)
{
const CvMat* prediction = cvKalmanPredict(kalman,0);

float x1= (float)prediction->data.fl[0];
float y1= (float)prediction->data.fl[1];

CvPoint2D32f predictionPt = cvPoint2D32f(x1,y1);

measurement->data.fl[0]=point.x;
measurement->data.fl[1] =point.y;

CvPoint measPt = cvPoint(measurement->data.fl[0],measurement->data.fl[1]);

mousev.push_back(measPt);

const CvMat* estimated= cvKalmanCorrect( kalman,measurement);

float x11= estimated->data.fl[0];
float y11 =estimated->data.fl[1];

printf("gandla");

CvPoint statePt=cvPoint(x11,y11);
kalmanv.push_back(statePt);

cvZero(img);

cvLine( img, cvPoint(statePt.x - 5, statePt.y - 5 ),cvPoint( statePt.x + 5, statePt.y + 5 ), cvScalar(255,255,255), 2,16);
cvLine( img, cvPoint(statePt.x + 5, statePt.y - 5 ),cvPoint( statePt.x - 5, statePt.y + 5 ), cvScalar(255,255,255), 2,16);

cvLine( img, cvPoint(measPt.x - 5, measPt.y - 5 ),cvPoint( measPt.x + 5, measPt.y + 5 ), cvScalar(255,255,255), 2,16);
cvLine( img, cvPoint(measPt.x + 5, measPt.y - 5 ),cvPoint( measPt.x - 5, measPt.y + 5 ), cvScalar(255,255,255), 2,16);

cv::Mat mat_img(img);
for (int i = 0; i < mousev.size()-1; i++)
{
cv::line(mat_img, mousev[i], mousev[i+1], cv::Scalar(255,255,0), 1);
}

cvWaitKey(0);
}
if( code == 27 || code == 'q' || code == 'Q' )
break;
}

return 0;
}

