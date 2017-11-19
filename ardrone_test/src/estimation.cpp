#include "ros/ros.h"
#include "ardrone_autonomy/Navdata.h"    //Accessing ardrone's published data
#include "ardrone_test/Drone_odo.h"
#include "ardrone_test/est_co.h"
#include <stdio.h>
#include <math.h>
#include <iostream>
#include <vector>

ardrone_test::Drone_odo val;
ardrone_test::est_co co_or;
ardrone_autonomy::Navdata drone_navdata;  	// Using this class variable for storing navdata

int redX,redY;
double t0, f= 100, z;
float i =0;
ros::Publisher est_pub;
ros::Subscriber pose_subscriber;		// ardrone navdata subsciber
ros::Subscriber num_sub;

int count = 0;

void NumCallback(const ardrone_test::Drone_odo::ConstPtr& val);
void poseCallback(const ardrone_autonomy::Navdata::ConstPtr & pose_message);		// drone actual data

int main(int argc, char** argv)
   {
     ros::init(argc, argv, "estimation");

     ros::NodeHandle nh_;
     pose_subscriber = nh_.subscribe("/ardrone/navdata", 10, poseCallback);
 	 num_sub = nh_.subscribe("chatter1", 1000, NumCallback);

      est_pub = nh_.advertise<ardrone_test::est_co>("est_odo", 1000);

      double ix = 0, iy =0, foc = 685, res =0;

  	ros::Rate loop_rate(f);

  	do{

  		ix = ix + drone_navdata.vx*0.001/f;
  		iy = iy + drone_navdata.vy*0.001/f;

  		co_or.x = ix;
  		co_or.y = iy;

  		est_pub.publish(co_or);

  		if(redX > 0 && res < 1)
  		{
  			z=drone_navdata.altd*0.001;
  			ix = (z/foc)*(180-redY);
  			iy = (z/foc)*(320-redX);
  			res =1;

  		}

  		ros::spinOnce(); //if this function is not mentioned the function will stay in the buffer and cannot be able to publish
  		loop_rate.sleep();

  	}while(1 && (ros::ok()));

     ros::spin();
     return 0;
   }

void poseCallback(const ardrone_autonomy::Navdata::ConstPtr & pose_message )
{
	drone_navdata.vx	= 	pose_message->vx;
	drone_navdata.vy 	= 	pose_message->vy;
}
void NumCallback(const ardrone_test::Drone_odo::ConstPtr& val)
{
  //cout<<"Target:"<< val->x<< val->y;
	redX = val->x;
	redY = val->y;
}
