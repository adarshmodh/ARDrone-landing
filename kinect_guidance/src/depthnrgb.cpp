
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>    // OpenCV
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>


//#define EXACT
#define APPROXIMATE


#ifdef EXACT
#include <message_filters/sync_policies/exact_time.h>
#endif
#ifdef APPROXIMATE
#include <message_filters/sync_policies/approximate_time.h>
#endif



using namespace std;
//using namespace sensor_msgs;
using namespace message_filters;


// Contador para la numeración de los archivos.
// Counter for filenames.
unsigned int cnt = 1;



// Handler / callback
void callback( const sensor_msgs::ImageConstPtr& msg_rgb , const sensor_msgs::ImageConstPtr& msg_depth )
{
        //ROS_INFO_STREAM("Adentro del callback\n");
      cv_bridge::CvImagePtr img_ptr_rgb;
        cv_bridge::CvImagePtr img_ptr_depth;
    try{
        img_ptr_depth = cv_bridge::toCvCopy(*msg_depth, sensor_msgs::image_encodings::TYPE_16UC1);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception:  %s", e.what());
        return;
    }
    try{
        img_ptr_rgb = cv_bridge::toCvCopy(*msg_rgb, sensor_msgs::image_encodings::TYPE_8UC3);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception:  %s", e.what());
        return;
    }

    	cv::Mat& mat_depth = img_ptr_depth->image;
        cv::Mat& mat_rgb = img_ptr_rgb->image;

}





int main(int argc, char** argv)
{
    // Initialize the ROS system and become a node.
  ros::init(argc, argv, "depthnrgb");
  ros::NodeHandle nh;


    message_filters::Subscriber<sensor_msgs::Image> subscriber_depth( nh , "/kinect2/qhd/image_depth_rect" , 1 );
    message_filters::Subscriber<sensor_msgs::Image> subscriber_rgb( nh , "/kinect2/qhd/image_color_rect" , 1 );


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


  return 0;
}

