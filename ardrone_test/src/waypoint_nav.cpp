/*
 * waypoint_nav.cpp
 *
 *  Created on: 30-Nov-2015
 *      Author: kiraa
 */
// Header files
#include "ros/ros.h"						//for sync ros
#include "geometry_msgs/Twist.h"			//for command velocity
#include "geometry_msgs/Vector3.h"			//for command velocity
#include "std_msgs/Empty.h"	 				//For take off and landing
#include "ardrone_autonomy/CamSelect.h"    	// For toggling camera
#include "ardrone_autonomy/Navdata.h"    //Accessing ardrone's published data
#include "sensor_msgs/Imu.h"

#include "ardrone_autonomy/navdata_gps.h"    //Accessing ardrone's published data
#include <sensor_msgs/Joy.h>
#include <stdio.h>
#include <math.h>
#include <termios.h>
#include <sstream>
#include <iostream>
#include <fstream>
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "ardrone_test/Drone_odo.h"
#include "ardrone_test/est_co.h"

// class variables
std_msgs::Empty emp_msg;				// variable in the take off and landing class
geometry_msgs::Twist vel_msg;			// variable in the command velocity class

// Variables for Publishing
ros::Publisher T_pub_empty;				//take off publisher
ros::Publisher L_pub_empty;				//landing publisher
ros::Publisher velocity_publisher;		// velocity publisher

// Variables for Subscribing
ros::Subscriber pose_subscriber;		// ardrone navdata subsciber
ros::Subscriber gps_subscriber;		// ardrone navdata subsciber
ros::Subscriber imu_subscriber;		// ardrone navdata subsciber
ros::Subscriber est_sub;		// ardrone navdata subsciber
ros::Subscriber INS_K_est;		// ardrone navdata subsciber
ros::Subscriber joy_sub_;
ros::Subscriber num_sub;
//ros::Subscriber filter_subscriber;		// tum estimation subscriber

// Variables for Service
ros::ServiceClient client;		// ardrone camera service

using namespace std;

float lx, ly, lz, ax, ay, az,foc = 685, angx,angy,angz, est_x, est_y, est_z,K_est_x, K_est_y, K_est_z;
int to, la, f = 5 , redX, redY, res =0, H=0;

double k = 0.5, z, lat, lon, ele, tf, ang_time; 						// k value for tau

//static float pos_x = 0, pos_y = 0, pos_z = 0;

const double PI = 3.14159265359;

double Kp = 0.2, Kd = 1,Kpz;							// Gain for the controller

double k1 = 2.438, k2 = 0.779, k3 = 1.234, k4 = 0.221;							// Gain for the controller

double tol = 0.4;							// error tolerance

ardrone_autonomy::Navdata drone_navdata;  	// Using this class variable for storing navdata
sensor_msgs::Imu ang;

void NumCallback(const ardrone_test::Drone_odo::ConstPtr& val);
void EstCallback(const ardrone_test::est_co::ConstPtr& est);
void INS_K_Callback(const ardrone_test::est_co::ConstPtr& esti);
void poseCallback(const ardrone_autonomy::Navdata::ConstPtr & pose_message);		// drone actual data
void gpsCallback(const ardrone_autonomy::navdata_gps::ConstPtr & gps_message);		// drone actual data
void imuCallback(const sensor_msgs::Imu::ConstPtr & imu_message);		// drone actual data
void joyCallback(const sensor_msgs::Joy::ConstPtr & joy);		// Joy data
void record();

// basic func
void hover();		// hovering
void takeoff();		// take off
void land();		//landing
void move(float lx, float ly, float lz, float ax, float ay, float az ); // publishing command velocity

// Basic Navigation func
void yawing(double angle);				// yawing movements

//Advanced Navigation function
void moveadv(double x_d, double y_d, double z_d);
void k_move(double x_d, double y_d, double z_d);
void k_hold();
void moveup(double z_d);
void validate(double x_d, double y_d, double z_d);
void tau(double x_d, double y_d, double z0);
void min_jerk(double x_d, double y_d, double z0, double tf);
//void itau(double x_d, double y_d, double z0);
void onTarget(int redX, int redY, double Vx, double Vy);
// Miscellaneous func
int getch();								//similar to getting input
double deg2rad(double angle_in_degrees);	//degree to radian converter
double getDistance(double x1, double y1, double z1, double x2, double y2, double z2);

int main(int argc, char **argv)
{
	//Initiate the ROS

	ros::init(argc, argv, "waypuoint_nav");
	ros::NodeHandle n; 			// Nodehandle is like a class where you are operating

	// Publishing the data
	velocity_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 100);

	T_pub_empty = n.advertise<std_msgs::Empty>("/ardrone/takeoff", 1); /* Message queue length is just 1 */

	L_pub_empty = n.advertise<std_msgs::Empty>("/ardrone/land", 1); /* Message queue length is just 1 */

	// Subscribing the data
	pose_subscriber = n.subscribe("/ardrone/navdata", 10, poseCallback);
	imu_subscriber = n.subscribe("/ardrone/imu", 10, imuCallback);
	gps_subscriber = n.subscribe("/ardrone/navdata_gps", 10, gpsCallback);
	est_sub = n.subscribe("Coordinate", 100, EstCallback);
	INS_K_est = n.subscribe("/estimation_data", 200, INS_K_Callback);

	num_sub = n.subscribe("/chatter1", 100, NumCallback);


	joy_sub_ = n.subscribe<sensor_msgs::Joy>("/joy", 10, joyCallback);

	// Calling toggle cam service

	client = n.serviceClient<ardrone_autonomy::CamSelect>("/ardrone/setcamchannel", 1);

	ardrone_autonomy::CamSelect srv;

	srv.request.channel = 1;
	if (client.call(srv))
	{
		ROS_INFO("Called");
	}
	else
	{
		ROS_INFO("Nope");
	}

	// Operations the actual main function after ros initialization
while(ros::ok())
{
    cout<<"Press any key"<<endl;

    cout<<"t -> takeoff"<<endl;
    cout<<"l -> land"<<endl;
    cout<<"a -> Goto a co - ordinate"<<endl;
    cout<<"n -> Record data"<<endl;
    cout<<"y -> yawing movements"<<endl;
    cout<<"h -> To hover in a place"<<endl;
    cout<<"p -> Tau"<<endl;
    cout<<"k -> Kinect Based"<<endl;
    cout<<"f -> Kinect Hold"<<endl;
    cout<<"j -> Min_jerk"<<endl;
    cout<<"i -> Image Tau"<<endl;
    cout<<"v -> Validate"<<endl;
    cout<<"sq -> Validate"<<endl;

    int c = getch();   // call your non-blocking input function

    double x, y, z;			// for reference value

    switch (c)
    {
	case 'a':
			cout<<endl<<"Height"<<endl;
			cin>>z;
			moveup(z);//tau(x,y,z);
			hover();
			//z = drone_navdata.altd*0.001;

    		cout<<endl<<"Co - ordinates"<<est_x<<" "<<est_y<<endl;
    		cin>>x>>y;
    		//z =0;
    		moveadv(x,y,0);//tau(x,y,z);
    		k_hold();//tau(x,y,z);
    		//moveadv(-x,0,0);//tau(x,y,z);
    		break;
	case 'k':
	//		cout<<endl<<"z Gain"<<endl;
		//	cin>>Kpz;
			cout<<endl<<"Height"<<endl;
			cin>>z;
			//cout<<endl<<"Co - ordinates"<<endl;
			//cin>>x>>y;
			//cout<<"co or"<<x<<setw(10)<<y<<endl;
			moveup(z);//tau(x,y,z);
			//hover();
			//z = drone_navdata.altd*0.001;

    		cout<<endl<<"Co - ordinates"<<endl;
    		cin>>x>>y;
      		cout<<endl<<"x & y Gain"<<endl;
      		cin>>Kp;
    		cout<<endl<<"Tol"<<endl;
       		cin>>tol;
    //    		Kp=0.1;
    		//z =0;
    		k_move(x,y,0);//tau(x,y,z);
    		//moveadv(-x,0,0);//tau(x,y,z);
    		break;
	case 'f':
	/*			cout<<endl<<"Height"<<endl;
				cin>>z;
				moveup(z);//tau(x,y,z);
				hover();
				//z = drone_navdata.altd*0.001;
	*/
	    		cout<<endl<<"Co - ordinates"<<endl;
	    		x = est_x;
	    		y = est_y;
	    		z = est_z;
	    		cout<<x<<setw(20)<<y<<endl;
	    		//z =0;
	    		//k_hold(x,y,0);//tau(x,y,z);
	    		//moveadv(-x,0,0);//tau(x,y,z);
	    		break;

	case 'q':
			cout<<endl<<"Height"<<endl;
			cin>>z;
			moveup(z);//tau(x,y,z);
//			hover();
			//z = drone_navdata.altd*0.001;
    		cout<<endl<<"Co - ordinates"<<K_est_x<<setw(20)<<K_est_y<<setw(20)<<K_est_z<<setw(20);
    		//z =0;
    		k_hold();//tau(x,y,z);
    		//moveadv(-x,0,0);//tau(x,y,z);
    		break;
	case 's':
				cout<<endl<<"UP"<<endl;
			    cin>>z;
			   // moveup(z);
	    		cout<<endl<<"side"<<endl;
	    		cin>>x;
	    		z =0;
	    		moveadv(x,0,0);
	    		hover();
	    		moveadv(0,x,0);
	    		hover();
	    		moveadv(-x,0,0);
	    		hover();
	    		moveadv(0,-x,0);
	    		hover();
	    		break;

	case 'p':
					cout<<endl<<"Height"<<endl;
					cin>>z;
		//			moveup(z);//tau(x,y,z);
					z = drone_navdata.altd*0.001;
				    cout<<setw(20)<<z;
				    cout<<endl<<"Co - ordinates"<<endl;
		    		cin>>x>>y;
		    		tau(x,y,z);
		    		break;
	case 'j':
					cout<<endl<<"Height"<<endl;
					cin>>z;
					//moveup(z);//tau(x,y,z);
					z = drone_navdata.altd*0.001;
				    cout<<setw(20)<<z;

				    cout<<endl<<"Co - ordinates"<<endl;
		    		cin>>x;
		    		cout<<endl<<"Final Time"<<endl;
		    		cin>>tf;
		    		hover();

		    		min_jerk(x,0,z,tf);
		    		break;

	case 'i':
					cout<<endl<<"Height"<<endl;
					cin>>z;
			//		moveup(z);//tau(x,y,z);
					z = drone_navdata.altd*0.001;
				    cout<<setw(20)<<z;
				    //itau(redX,redY,z);
		    		break;

	case 'v':
    		cout<<endl<<"Co - ordinates"<<endl;
    		cin>>x;
    		//z =0;
    		validate(x,0,0);//tau(x,y,z);
    		//moveadv(-x,0,0);//tau(x,y,z);
    		break;

    	case 'h':
    		cout<<"Hovering in a place"<<endl;
    		hover();
    		break;
    	case 'l':
    		cout<<endl<<"Landing initiated"<<endl;
    		land();
    		break;
    	case 't':
    		cout<<endl<<"Take off initiated"<<endl;
    		takeoff();
    		hover();
    		break;

    		/*For checking navdata working*/
    	  case 'n':
    		  record();
    		  /*cout<<endl<<"Co - ordinates"<<endl;
    		   cin>>x>>y;
    		   z =0;
    		   moveadv(x,y,0);//tau(x,y,z);
    		   */
    		  break;
    	case 'y':
    		cout<<endl<<"Rotation in degree :"<<endl;
    		cin>>x;
    		yawing(x);
    		break;
    	default:
    		land();
    		break;

    }
    hover();
}
land();
cout<<"landed";

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
	ang.linear_acceleration =  imu_message->linear_acceleration;
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
	est_x = -est->z;
	est_y = est->y;
	est_z = est->x;
	//cout<<"Target:"<< est_x<< setw(25)<<est_y<<setw(25) << est_z;
}

void INS_K_Callback(const ardrone_test::est_co::ConstPtr& esti)
{
  //cout<<"Target:"<< val->x<< val->y;
	K_est_x = esti->x;
	K_est_y = esti->y;
	K_est_z = esti->z;
	ROS_INFO("hi = %f, %f ",K_est_x,K_est_y);
	//cout<<"Target:"<< est_x<< setw(25)<<est_y<<setw(25) << est_z;
}


void joyCallback(const sensor_msgs::Joy::ConstPtr & joy )
{
	lx = joy->axes[0];
	ly = joy->axes[1];
	az = joy->axes[2];
	lz = joy->axes[3];
	to = joy->buttons[8];
	la = joy->buttons[9];
}

int getch()
{
  static struct termios oldt, newt;
  tcgetattr( STDIN_FILENO, &oldt);           // save old settings
  newt = oldt;
  newt.c_lflag &= ~(ICANON);                 // disable buffering
  tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings

  int c = getchar();  // read character (non-blocking)

  tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
  return c;
}

double deg2rad(double angle_in_degrees)
{
	return angle_in_degrees*PI/180.0;
}

double getDistance(double x1, double y1, double z1, double x2, double y2, double z2)
{
	return sqrt(pow(x1-x2, 2)+pow(y1-y2, 2)+pow(z1-z2, 2));
}

void takeoff()
{
	ros::Rate loop_rate(10);
	while (ros::ok())
		{
			double init_time=ros::Time::now().toSec();
			double time;
			while (time < (init_time+5.0)) /* Send command for five seconds*/
			{
				T_pub_empty.publish(emp_msg); /* launches the drone */
				ros::spinOnce();
				loop_rate.sleep();
				time = ros::Time::now().toSec();
			}//time loop
		ROS_INFO("ARdrone launched");
		exit(0);
		}//ros::ok loop
	hover();

}

void land()
{
	ros::Rate loop_rate(10);
	while (ros::ok())
		{
			double init_time=ros::Time::now().toSec();
			double time;
			while (time < (init_time+5.0)) /* Send command for five seconds*/
			{
				L_pub_empty.publish(emp_msg); /* launches the drone */
				ros::spinOnce();
				loop_rate.sleep();
				time = ros::Time::now().toSec();
			}//time loop
		ROS_INFO("ARdrone landed");
		exit(0);
		}//ros::ok loop

}

void hover()
{

	double t0 = ros::Time::now().toSec(); //ros has a 'Time' function in which the current time can be found by using 'now'.                                         							we need time in sec to compute hence 'tosec'
	double t1;

	ros::Rate loop_rate(10);

	do{

		t1 = ros::Time::now().toSec(); //ros has a 'Time' function in which the current time can be found by using 'now'.                                         							we need time in sec to compute hence 'tosec'

		vel_msg.linear.x = 0;
		vel_msg.linear.y = 0;
		vel_msg.linear.z = 0;

	vel_msg.angular.x = 0;
	vel_msg.angular.y = 0;
	vel_msg.angular.z = 0;

	velocity_publisher.publish(vel_msg);

	ros::spinOnce(); //if this function is not mentioned the function will stay in the buffer and cannot be able to publish
	loop_rate.sleep();

	}while(t1 <= (t0+3));

	//ros::spinOnce();
}

void move(float lx, float ly, float lz, float ax, float ay, float az )
{

	//defining the linear velocity
	vel_msg.linear.x = lx;
	vel_msg.linear.y = ly;
	vel_msg.linear.z = lz;

	//defining the linear velocity
	vel_msg.angular.x = 0;
	vel_msg.angular.y = 0;
	vel_msg.angular.z = az;

	velocity_publisher.publish(vel_msg);

}

void record()
{

	double i =0, dt = 0, a, b, c, d;
	//float xang = 0, yang= 0, zang=0, exang, eyang, ezang, rotX, rotY, rotZ, angX, angY, angZ;

	double t0=ros::Time::now().toSec();

		ofstream myfile;
		myfile.open("/home/icgel/Cali.txt", ios::out | ios::app );

		ros::Rate loop_rate(f);

		do{
			double t1=ros::Time::now().toSec();

			dt = t1-t0;

			myfile<<endl<<i<<setw(20);
			myfile<<dt<<setw(20);
			/*
			myfile<<a<<setw(20);
			myfile<<b<<setw(20);
			myfile<<c<<setw(20);
			myfile<<d<<setw(20);

			myfile<<drone_navdata.vx<<setw(20);
			myfile<<drone_navdata.vy<<setw(20);
			myfile<<drone_navdata.altd<<setw(20);

			myfile<<drone_navdata.ax<<setw(20);
			myfile<<drone_navdata.ay<<setw(20);
			myfile<<drone_navdata.az<<setw(20);


			rotX = drone_navdata.rotX;
			angX = angx;
			rotY = drone_navdata.rotY;
			angY = angy;
			rotZ = drone_navdata.rotZ;
			angZ = angz;

			myfile<<lat<<setw(20);
			myfile<<lon<<setw(20);
			myfile<<ele<<setw(20);
*/
			myfile<<ang.angular_velocity.x<<setw(30);
			myfile<<ang.angular_velocity.y<<setw(30);
			myfile<<ang.angular_velocity.z<<setw(30);

			myfile<<ang.linear_acceleration.x<<setw(30);
			myfile<<ang.linear_acceleration.y<<setw(30);
			myfile<<ang.linear_acceleration.z;
/*			cout<<exang<<endl;
			cout<<eyang<<endl;
			cout<<ezang<<endl;
			xang = xang + angX*dt;
			yang = yang + angY*dt;
			zang = zang + angZ*dt;
			exang = rotX - xang;
			eyang = rotY - yang;
			ezang = rotZ - zang;
*/
			//myfile<<drone_navdata.tm<<setw(40);
			//myfile<<drone_navdata.altd*0.001;

			i = i+1;

			t0 = t1;

			ros::spinOnce(); //if this function is not mentioned the function will stay in the buffer and cannot be able to publish
			loop_rate.sleep();
		}while(ros::ok());
		myfile.close();
/*
	while(ros::ok())
	{
		//cout<<"Target:"<< redX<< setw(25)<<redY<<endl;
		cout<<"Target:"<< est_x<< setw(25)<<est_y<<setw(25) << est_z<<endl;
		ros::spinOnce(); //if this function is not mentioned the function will stay in the buffer and cannot be able to publish

	}
	*/
}

void tau(double x_d, double y_d, double z0)
{
	ax = 0;
	ay = 0;
	//az = 0;

	//Measuring t : current time
	double t0 = ros::Time::now().toSec(); //ros has a 'Time' function in which the current time can be found by using 'now'.                                         							we need time in sec to compute hence 'tosec'

	double e_x, e_y, e_z,e_x0, e_y0, e_z0, x=0, y=0,z,r, i=0,v,phi, error, tau1, nr;

	v = 0.5;

	double r0 = r, iphi, a, x0, y0, dxg, dyg, dzg, dxg0= 0, dyg0 = 0, dzg0=0, axg= 0, ayg = 0, azg=0, xg = 0, yg = 0, zg = z0, xg0 = x_d, yg0 = y_d, zg0 = z0;
	e_x0 = x-x_d;
	e_y0 = y-y_d;
	e_z0 = z0;

	r0 = getDistance(e_x0, e_y0, e_z0, 0, 0, 0);

	e_x = e_x0;
	e_y = e_y0;
	e_z = e_z0;

	tau1 = -r0/v;

	nr = getDistance(x_d, y_d, 0, 0, 0, 0);
	iphi = asin(nr/r0);

	ros::Rate loop_rate(f);

	ofstream myfile;
	myfile.open("/home/icgel/tau_test.txt", ios::out | ios::app );
	//myfile<<"Entering the loop";

	z = z0;
	r = r0;

	do{
		double t1=ros::Time::now().toSec();

		//cout<<endl<<"ALT = "<<z<<endl;

		e_x = e_x0*pow((1+k*(t1-t0)/tau1),(1/k));
		e_y = e_y0*pow((1+k*(t1-t0)/tau1),(1/k));
		e_z = e_z0*pow((1+k*(t1-t0)/tau1),(1/k));

		r = getDistance(e_x, e_y, e_z, 0, 0, 0);

		phi = iphi * pow((r/r0),(1/k));

		a = r*sin(phi)/(r0*sin(iphi));

		xg = (1-a)* x_d;
		yg = (1-a)* y_d;
		zg = e_z;

		//cout<<endl<<"Time = "<<t1-t0<<endl;

/*		myfile<<xg<<setw(20);
		myfile<<yg<<setw(20);
		myfile<<zg<<setw(20);
*/
		//cout<<x<<setw(20)<<xg<<endl<<y<<setw(20)<<yg<<endl<<z<<setw(20)<<zg<<endl;

		//cout<<endl<<"Dist = "<<r<<endl;

		dxg = (xg-xg0)*f; //f = 1/dt;
		dyg = (yg-yg0)*f;
		dzg = (zg-zg0)*f;

		axg = (dxg-dxg0)*f;
		ayg = (dyg-dyg0)*f;
		azg = (dzg-dzg0)*f;

		if(z>0.1 || i<2)
		{
			//lx = 1/k1*(k2*drone_navdata.vx*0.001 + axg - Kd*0.3*(drone_navdata.vx*0.001-dxg) - Kp*(x-xg));
			//ly = 1/k3*(k4*drone_navdata.vy*0.001 + ayg - Kd*0.3*(drone_navdata.vy*0.001-dyg) - Kp*(y-yg));
			lx = - Kd*0.3*(drone_navdata.vx*0.001-dxg) - Kp*0.8*(x-xg);
			ly = - Kd*0.3*(drone_navdata.vy*0.001-dyg) - Kp*0.8*(y-yg);
			lz = -((0.7)*(z-zg));
			//cout<<"Cmd_vel"<<lz<<endl;
		}
		else
		{
			lx = 0;
			ly = 0;
			lz =0;
			break;
		}

		xg0 = xg;
		yg0 = yg;
		zg0 = zg;

		dxg0 = dxg;
		dyg0 = dyg;
		dzg0 = dzg;

		i = i+1;
		//cout<<"Iteration"<<i<<endl;
		myfile<<endl<<i<<setw(25);


		move(lx, ly, lz, ax, ay, az);
/*
		if(res ==0 && redX > 0 )
		{
			int iredX= redX;
			int iredY = redY;
			double Vx = drone_navdata.vx;
			double Vy = drone_navdata.vy;

			cout<<"Interrupt called"<<redX<<setw(10)<<drone_navdata.vx<<endl;
			hover();
			onTarget(redX, redY, Vx, Vy);
			break;
		}
*/


		myfile<<x<<setw(25);
		myfile<<y<<setw(25);
		myfile<<z<<setw(25);

		myfile<<xg<<setw(25);
		myfile<<yg<<setw(25);
		myfile<<zg<<setw(25);

		myfile<<drone_navdata.vx<<setw(25);
		myfile<<drone_navdata.vy<<setw(25);
		myfile<<drone_navdata.altd<<setw(25);

		myfile<<dxg<<setw(25);
		myfile<<dyg<<setw(25);
		myfile<<dzg<<setw(25);

		myfile<<drone_navdata.ax<<setw(25);
		myfile<<drone_navdata.ay<<setw(25);
		myfile<<drone_navdata.az<<setw(25);

		myfile<<axg<<setw(25);
		myfile<<ayg<<setw(25);
		myfile<<azg;

		x = x + drone_navdata.vx*0.001/f;
		y = y + drone_navdata.vy*0.001/f;
		z = drone_navdata.altd*0.001;


		ros::spinOnce(); //if this function is not mentioned the function will stay in the buffer and cannot be able to publish
		loop_rate.sleep();


		}while((r >= 0.3) && (ros::ok()));
		myfile.close();

}



void yawing(double angle)
{
	angle = angle + drone_navdata.rotZ;

		double error;
		ros::Rate loop_rate(10);
		do{

			//velocity_publisher.publish(vel_msg);

			error = (angle - drone_navdata.rotZ);
			cout<< "Yawwwwwwwwwwwwwww"<<drone_navdata.rotZ<<endl;
			vel_msg.angular.z = 0.1 * deg2rad(error);

			ros::spinOnce(); //if this function is not mentioned the function will stay in the buffer and cannot be able to publish
			loop_rate.sleep();

		}while((abs(error) > 5) && (ros::ok()));

		vel_msg.angular.z = 0;
		velocity_publisher.publish(vel_msg);
}

void validate(double x_d, double y_d, double z_d)
{
	ax = 0;
	ay = 0;
	//az = 0;

	//Measuring t : current time
	double t1, t0 = ros::Time::now().toSec(); //ros has a 'Time' function in which the current time can be found by using 'now'.                                         							we need time in sec to compute hence 'tosec'

	double e_x, e_y = 0, e_z, error, i = 0;

	ros::Rate loop_rate(f);

	ofstream myfile;
	myfile.open("/home/icgel/vali.txt", ios::out | ios::app );
	//myfile<<"Entering the loop";


	do{
		t1=ros::Time::now().toSec();

	   //e_x = drone_navdata.vx* (t1-t0)*0.01 - x_d;//ASHU convert to inertial frame
	   //e_y = drone_navdata.vy* (t1-t0)*0.01 - y_d;

	   //double t2=ros::Time::now().toSec();
	   //cout<<"Time Stamps between the data = "<<t2-t1<<endl;

		//error = getDistance(0, 0, e_x, e_y);// only for tolernace

		myfile<<endl<<i<<setw(20);
		myfile<<lat<<setw(20);
		myfile<<lon<<setw(20);
		myfile<<ele<<setw(20);
		myfile<<drone_navdata.ax<<setw(20);
		myfile<<drone_navdata.ay<<setw(20);
		myfile<<drone_navdata.az<<setw(20);
		myfile<<ang.angular_velocity.x<<setw(20);
		myfile<<ang.angular_velocity.y<<setw(20);
		myfile<<ang.angular_velocity.z;

		//myfile<<drone_navdata.vx<<endl;
		//myfile<<drone_navdata.vy<<endl;
		//myfile<<drone_navdata.vz<<endl;


	    if (ros::ok())
		{
	    	//myfile<<t1-t0<<setw(10);
	    	cout<<"time = "<<t1-t0<<endl;
	    	lx = -Kp*2*(drone_navdata.vx*0.001-x_d);
	    	ly = -Kp*(drone_navdata.vy*0.001);
	    	//lx = 0;
	    	//ly =0;
	    	lz = 0;

//			myfile<<lx<<setw(10);
	//		cout<<"u_phi = "<<lx<<endl;
	    				//lz = Kp * e_z;
	     }
		else
		{
	  //  	myfile<<t1-t0<<setw(10);
			lx = 0;
			ly =0;
			lz = 0;
		//	myfile<<ly<<setw(10);
			//cout<<"u_phi = "<<ly<<endl;

		}


		//t0 = t1;

		i = i +1;

		cout<<"iteration"<<i<<endl;;


		move(lx, ly, lz, ax, ay, az );
		//myfile<<endl<<vel_msg.linear.x<<endl;
	    //myfile<<endl<<vel_msg.linear.y<<endl;


		ros::spinOnce(); //if this function is not mentioned the function will stay in the buffer and cannot be able to publish
		loop_rate.sleep();


	}while(((t1-t0 < 120)) && (ros::ok()));
	myfile.close();


}


void moveadv(double x, double y, double z)
{

	cout<<x<<" "<<" "<<y<<endl;
	ax = 0;
	ay = 0;
	//az = 0;

	//Measuring t : current time
	double t0 = ros::Time::now().toSec(); //ros has a 'Time' function in which the current time can be found by using 'now'.                                         							we need time in sec to compute hence 'tosec'

	double e_x, e_y, e_z,r,r0, i=0,z0, Vx, Vy, Vx0=0, Vy0=0,x_d =0,y_d =0,z_d =0;
	z0 = drone_navdata.altd*0.001;


	r0 = getDistance(0, 0, 0, x, y, z);

	ros::Rate loop_rate(f);

	ofstream myfile;
	myfile.open("/home/icgel/mov_test.txt", ios::out | ios::app );
	//myfile<<"Entering the loop";

	z = z0;
	r = r0;

	do{
		double t1=ros::Time::now().toSec();

		//cout<<endl<<"ALT = "<<z<<endl;

		e_x = x - x_d;
		e_y = y - y_d;
		e_z = 0;

		if(r > 0.25)
		{
			Vx = (drone_navdata.vx*0.001);
			Vy = (drone_navdata.vy*0.001);
			lx =  - Kp*0.25*(e_x)- Kd*0.05*Vx;
			ly = - Kp*0.25*(e_y)- Kd*0.05*Vy;
			lz = 0;

			cout<<"Cmd_vel"<<lx<<setw(10)<<ly<<setw(10)<<lz<<endl;
		}
		else
		{
			lx = 0;
			ly = 0;
			lz = 0;
			break;
		}

		i = i+1;
		//cout<<x<<setw(20)<<y<<setw(20)<<z<<endl;
//		cout<<"Iteration"<<i<<endl;
		myfile<<endl<<i<<setw(20);
/*
		if(res == 0 && redX > 0 )
		{
			int iredX= redX;
			int iredY = redY;
			Vx = drone_navdata.vx;
			Vy = drone_navdata.vy;

			cout<<"Interrupt called"<<redX<<setw(10)<<drone_navdata.vx<<endl;
			hover();
			//move(0,0,0,0,0,0);
			onTarget(redX, redY, Vx, Vy);
			break;
		}
*/
		//myfile<<lx<<setw(20);
		//myfile<<ly<<setw(20);
		//myfile<<lz<<setw(20);


		myfile<<drone_navdata.header.stamp<<setw(50);

		move(lx, ly, lz, ax, ay, az);

	//	cout<<"co or"<<x<<setw(10)<<y<<setw(10)<<z<<endl;
/*
		myfile<<drone_navdata.vx*0.001<<setw(25);
		myfile<<drone_navdata.vy*0.001<<setw(25);
		myfile<<drone_navdata.altd*0.001<<setw(25);
		myfile<<drone_navdata.rotX<<setw(25);
		myfile<<drone_navdata.rotY<<setw(25);
		myfile<<drone_navdata.rotZ<<setw(25);
		myfile<<drone_navdata.ax<<setw(25);
		myfile<<drone_navdata.ay<<setw(25);
		myfile<<drone_navdata.az<<setw(25);
		myfile<<ang.header.stamp<<setw(50);
		myfile<<ang.angular_velocity.x<<setw(25);
		myfile<<ang.angular_velocity.y<<setw(25);
		myfile<<ang.angular_velocity.z<<setw(25);
*/


		x = K_est_x;
		y = K_est_y;
		//x = x + ((Vx+Vx0)/2)/f;
		//y = y + ((Vy+Vy0)/2)/f;
		z = drone_navdata.altd*0.001;
		Vx0 = Vx;
		Vy0 = Vy;

		r = getDistance(e_x, e_y, e_z, 0,0,0);

		ros::spinOnce(); //if this function is not mentioned the function will stay in the buffer and cannot be able to publish
		loop_rate.sleep();


		}while((r > 0.25) && (ros::ok()));
		myfile.close();

}

void moveup(double z_d)
{
	//Measuring t : current time
	double t0 = ros::Time::now().toSec(); //ros has a 'Time' function in which the current time can be found by using 'now'.                                         							we need time in sec to compute hence 'tosec'

	double e_z, z =0, i=0 ,z0 ,r = 1, x, y, x0, y0, Vx, Vy, e_x, e_y;

	z0 = drone_navdata.altd*0.001;

	ros::Rate loop_rate(f);

	z = z0;
	ofstream myfile;
	myfile.open("/home/icgel/mov_test.txt", ios::out | ios::app );
	//myfile<<"Entering the loop";

	do{
		e_z = z - z_d;

		double t1=ros::Time::now().toSec();

		if((abs(e_z) > 0.5 && z > 0))
		{
			double z = drone_navdata.altd*0.001;
					lx = 0;
					ly = 0;
					lz = -0.7*(z-z_d);
			cout<<"Cmd_vel"<<lx<<setw(10)<<ly<<setw(10)<<lz<<endl;
		}
		else
		{
			lx =0;
			ly =0;
			lz = 0;
			//break;
		}

		myfile<<endl<<i<<setw(20);
		myfile<<drone_navdata.header.stamp<<setw(50);

		cout<<"iter"<<i<<endl;

		move(lx, ly, lz, 0, 0, 0);

		z = drone_navdata.altd*0.001;
/*
		myfile<<drone_navdata.vx*0.001<<setw(25);
		myfile<<drone_navdata.vy*0.001<<setw(25);
		myfile<<drone_navdata.altd*0.001<<setw(25);
		myfile<<drone_navdata.rotX<<setw(25);
		myfile<<drone_navdata.rotY<<setw(25);
		myfile<<drone_navdata.rotZ<<setw(25);
		myfile<<drone_navdata.ax<<setw(25);
		myfile<<drone_navdata.ay<<setw(25);
		myfile<<drone_navdata.az<<setw(25);
		myfile<<ang.header.stamp<<setw(50);
		myfile<<ang.angular_velocity.x<<setw(25);
		myfile<<ang.angular_velocity.y<<setw(25);
		myfile<<ang.angular_velocity.z<<setw(25);
*/

		i = i+1;

		ros::spinOnce(); //if this function is not mentioned the function will stay in the buffer and cannot be able to publish
		loop_rate.sleep();

		}while(abs(e_z)>0.5 && ros::ok());
		myfile.close();

}

void onTarget(int iredX, int iredY, double Vx, double Vy)
{

	res = 1;
	//hover();

	if(redX > 0)
	{
		double z=drone_navdata.altd*0.001;

		double x = (z/foc)*(180-redY);
		double y = (z/foc)*(320-redX);
		cout<<"Desired Pos"<<x<<"&"<<y<<endl;
	//	moveadv(-2,0,0);
//		cout<<"Est Pos"<<est_x<<"&"<<est_y<<endl;
		tau(x,y,z);
		land();
	}
	else
	{
		hover();
	}

//	ros::Rate loop_rate(f);
/*
	do
	{
		if(redX > 0)
			{
				double z = drone_navdata.altd*0.001;
				ly = -Kp*(redY - 320)*0.001;
				cout<<"On Target Move forward by "<<redX<<endl;
				lx = -Kp*(redX - 170)*0.001;
				cout<<"On Target Move right by "<<redY<<endl;
     			lz = 0;//-0.01*(z-1);
				//cout<<"On Target Move height by "<<z<<endl;

				Vx = drone_navdata.vx;
				Vy = drone_navdata.vy;
				z = drone_navdata.altd;

			}
		else
			{
				double z = drone_navdata.altd*0.001;
				lx = 0;//-Kp*0.001*Vx;
				//cout<<"Target Not found Move forward by "<<Vx<<endl;
				ly = 0;//-Kp*0.001*Vy;
				//cout<<"Target Not found Move right by "<<Vy<<endl;
				lz = 0; //-Kp*(z_new - z)*0.01;
			}

		move(lx,ly, lz, 0, 0, 0);

		ros::spinOnce(); //if this function is not mentioned the function will stay in the buffer and cannot be able to publish
		loop_rate.sleep();
	//}while((z > 1) && (ros::ok()));
	}while(((redY-320)> 25 || abs(redX-180)>25) && (ros::ok()));
	*/
	/*if(drone_navdata.altd*0.001 >1.7)
	{
		moveup(3.5);
		onTarget(redX, redY, drone_navdata.vx, drone_navdata.vy);
	}
	else
	{
		land();
	}
*/

}
void min_jerk(double x_d, double y_d, double z0, double Tf)
{
	ax = 0;
	ay = 0;
	//az = 0;

	//Measuring t : current time
	double t0 = ros::Time::now().toSec();
	double e_x, e_y, e_z,e_x0, e_y0, e_z0, x=0, y=0,z,r, i=0,v;


	double g,x0,u0,v0,ax0,az0,xf,zf,uf,vf,axf,azf, tgo, u1, u2,r0,c1x,c2x,c3x,c1z,c2z,c3z,K = 1, dxg, dxg0,dzg,dzg0,xg,xg0,zg,zg0, zi0;

	e_x0 = x-x_d;
	e_y0 = y-y_d;
	e_z0 = z0;

	r = getDistance(e_x0, e_y0, e_z0, 0, 0, 0);

	//cout<<endl<<r0;

	ros::Rate loop_rate(f);

	ofstream myfile;
	myfile.open("/home/icgel/jerk_test.txt", ios::out | ios::app );
	//myfile<<"Entering the loop";
/*
	zi0 =drone_navdata.altd*0.001;
	v0 =0;
	do{
			double t1=ros::Time::now().toSec();

			if(z > 1))
			{
				lx = 0;
				ly = 0;
				lz = -(Kd*(v0-0.3));
				cout<<"Cmd_vel"<<v0<<endl;
			}
			else
			{
				lx = 0;
				ly = 0;
				lz =0;
				break;
			}


			z = drone_navdata.altd*0.001;

			v0 = (z-zi0)*f;

			zi0 =z;

			ros::spinOnce(); //if this function is not mentioned the function will stay in the buffer and cannot be able to publish
			loop_rate.sleep();


			}while((v0<0.3) && (ros::ok()));

	z = drone_navdata.altd*0.001;

cout<<endl<<v0<<endl;
cout<<endl<<z<<endl;
*/
g=9.81;
x0=0;
u0=drone_navdata.vx*0.001;
v0=0;
ax0=0;
az0=-g;
xf=x_d;
zf=0.5;
uf=0;
vf=-0.5;
axf=0;
azf=-g;
dxg0 = 0;
dzg0 = v0;
xg0 = 0;
zg0=z0;
zg =zg0;

do
{
	if(redX > 0)
		{
			double z = drone_navdata.altd*0.001;
			ly = -Kp*(redY - 320)*0.001;
			cout<<"On Target Move forward by "<<redX<<endl;
			lx = -Kp*(redX - 170)*0.001;
			cout<<"On Target Move right by "<<redY<<endl;
 			lz = 0;//-0.01*(z-1);
			//cout<<"On Target Move height by "<<z<<endl;

		}
	else
		{
			double z = drone_navdata.altd*0.001;
			lx = 0;//-Kp*0.001*Vx;
			//cout<<"Target Not found Move forward by "<<Vx<<endl;
			ly = 0;//-Kp*0.001*Vy;
			//cout<<"Target Not found Move right by "<<Vy<<endl;
			lz = 0; //-Kp*(z_new - z)*0.01;
		}

	move(lx,ly, lz, 0, 0, 0);

	ros::spinOnce(); //if this function is not mentioned the function will stay in the buffer and cannot be able to publish
	loop_rate.sleep();
//}while((z > 1) && (ros::ok()));
}while(((redY-320)> 25 || abs(redX-180)>25) && (ros::ok()));


	do{
		double t1=ros::Time::now().toSec();

		c1x= (10/(Tf*Tf*Tf))*(x0-xf+uf*Tf-(axf/2)*Tf*Tf)+(4/(Tf*Tf))*(u0-uf+axf*Tf)+(1/(2*Tf))*(ax0-axf);
		//cout<<endl<<"coeff = "<<c1x<<endl;
		c2x= (-15/(Tf*Tf*Tf*Tf))*(x0-xf+uf*Tf-(axf/2)*Tf*Tf)-(7/(Tf*Tf*Tf))*(u0-uf+axf*Tf)-(1/(Tf*Tf))*(ax0-axf);
		//cout<<endl<<"coeff = "<<c2x<<endl;
		c3x= (6/(Tf*Tf*Tf*Tf*Tf))*(x0-xf+uf*Tf-(axf/2)*Tf*Tf)+(3/(Tf*Tf*Tf*Tf))*(u0-uf+axf*Tf)+(1/(2*Tf*Tf*Tf))*(ax0-axf);
		//cout<<endl<<"coeff = "<<c3x<<endl;


		c1z= (10/(Tf*Tf*Tf))*(z0-zf+vf*Tf-((azf+g)/2)*Tf*Tf)+(4/(Tf*Tf))*(v0-vf+(azf+g)*Tf)+(1/(2*Tf))*(az0-azf);
		//cout<<endl<<"coeff = "<<c1z<<endl;
		c2z= (-15/(Tf*Tf*Tf*Tf))*(z0-zf+vf*Tf-((azf+g)/2)*Tf*Tf)-(7/(Tf*Tf*Tf))*(v0-vf+(azf+g)*Tf)-(1/(Tf*Tf))*(az0-azf);
		//cout<<endl<<"coeff = "<<c2z<<endl;
		c3z= (6/(Tf*Tf*Tf*Tf*Tf))*(z0-zf+vf*Tf-((azf+g)/2)*Tf*Tf)+(3/(Tf*Tf*Tf*Tf))*(v0-vf+(azf+g)*Tf)+(1/(2*Tf*Tf*Tf))*(az0-azf);
		//cout<<endl<<"coeff = "<<c3z<<endl;
		tgo=Tf-(i)/f;

		cout<<endl<<tgo<<endl;
		cout<<endl<<zg<<endl;


		u1=20*c3x*tgo*tgo*tgo+12*c2x*tgo*tgo+6*c1x*tgo+axf;
		//cout<<endl<<u1;

		u2=20*c3z*tgo*tgo*tgo+12*c2z*tgo*tgo+6*c1z*tgo+azf;
		//cout<<endl<<u2;

		dxg = dxg0 + u1/f;
		dzg =  dzg0 + (u2+g)/f;

		xg = xg0 + dxg/f;
		zg =  zg0 + (dzg)/f;

		//cout<<endl<<"Time = "<<t1-t0<<endl;

		myfile<<xg<<setw(20);
		//myfile<<yg<<setw(20);
		myfile<<zg<<setw(20);
		myfile<<x<<setw(25);
		myfile<<y<<setw(25);
		myfile<<z<<setw(25);

		//cout<<x<<setw(20)<<xg<<endl<<y<<setw(20)<<yg<<endl<<z<<setw(20)<<zg<<endl;

		//cout<<endl<<"Dist = "<<r<<endl;



		if(r>0.5)
		{
			lx = - Kd*0.3*(drone_navdata.vx*0.001-dxg) - Kp*0.8*(x-xg);
			ly = 0;
			lz = -((0.7)*(z-zg));
			cout<<"Cmd_vel"<<lx<<endl;
		}
		else
		{
			lx = 0;
			ly = 0;
			lz =0;
			break;
		}



		i = i+1;
		//cout<<"Iteration"<<i<<endl;
		myfile<<endl<<i<<setw(25);


		move(lx, ly, lz, ax, ay, az);




		myfile<<drone_navdata.vx<<setw(25);
		myfile<<drone_navdata.vy<<setw(25);
		myfile<<drone_navdata.altd<<setw(25);

/*
		myfile<<drone_navdata.ax<<setw(25);
		myfile<<drone_navdata.ay<<setw(25);
		myfile<<drone_navdata.az<<setw(25);
*/


		x = x + drone_navdata.vx*0.001/f;
		y = y + drone_navdata.vy*0.001/f;
		z = drone_navdata.altd*0.001;

		dxg0 = dxg;
		dzg0 = dzg;
		xg0 = xg;
		zg0 = zg;

		r = getDistance(x, y, z, x_d, y_d, 0);


		ros::spinOnce(); //if this function is not mentioned the function will stay in the buffer and cannot be able to publish
		loop_rate.sleep();


		}while((zg >= 0.1) && (ros::ok()));
		myfile.close();

}

void k_move(double x_d, double y_d, double z_d)
{
	ax = 0;
	ay = 0;
	//az = 0;

	//Measuring t : current time
	double t0 = ros::Time::now().toSec(); //ros has a 'Time' function in which the current time can be found by using 'now'.                                         							we need time in sec to compute hence 'tosec'

	double e_x=0, e_y=0, e_z=0, x=0, y=0,z =0,r,r0, i=0,z0, Vx, Vy, Vx0=0, Vy0=0;
	z0 = drone_navdata.altd*0.001;


	r0 = getDistance(0, 0, 0, x_d, y_d, 0);

	ros::Rate loop_rate(f);

	ofstream myfile;
	myfile.open("/home/icgel/kinect_mov.txt", ios::out | ios::app );
	//myfile<<"Entering the loop";

	z = z0;
	r = 0;   //Must change initial value of r

	do{
		double t1=ros::Time::now().toSec();

		//cout<<endl<<"ALT = "<<z<<endl;

		e_x = x - x_d;
		e_y = y - y_d;
		e_z = z - z_d;

		if(r > tol)
		{
			if (redX==0 || redY==0)
			{
				lx = 0;
				ly = 0;
				lz = 0;

			}
			else
			{
				Vx = (drone_navdata.vx*0.001);
				Vy = (drone_navdata.vy*0.001);
				lx =  - Kp*0.2*(e_x)-Kd*0.025*(Vx);
				ly = - Kp*0.2*(e_y)-Kd*0.025*(Vy);
				lz = 0;//- Kpz*(e_z);
			}

			cout<<"Cmd_vel"<<lx<<setw(10)<<ly<<setw(10)<<lz<<endl;
		}
		else
		{
			lx = 0;
			ly = 0;
			lz = 0;
			//break;
		}

		i = i+1;
		//cout<<x<<setw(20)<<y<<setw(20)<<z<<endl;
//		cout<<"Iteration"<<i<<endl;
		myfile<<endl<<i<<setw(20);

		move(lx, ly, lz, ax, ay, az);
		cout<<"co or"<<x<<setw(10)<<y<<setw(10)<<z<<endl;
/*
		myfile<<drone_navdata.header.stamp<<setw(50);

		myfile<<drone_navdata.vx*0.001<<setw(25);
		myfile<<drone_navdata.vy*0.001<<setw(25);
		myfile<<drone_navdata.altd*0.001<<setw(25);
		myfile<<drone_navdata.rotX<<setw(25);
		myfile<<drone_navdata.rotY<<setw(25);
		myfile<<drone_navdata.rotZ<<setw(25);
		myfile<<drone_navdata.ax<<setw(25);
		myfile<<drone_navdata.ay<<setw(25);
		myfile<<drone_navdata.az<<setw(25);
		myfile<<ang.header.stamp<<setw(50);
		myfile<<ang.angular_velocity.x<<setw(25);
		myfile<<ang.angular_velocity.y<<setw(25);
		myfile<<ang.angular_velocity.z<<setw(25);
*/

		x = est_x; 				//x + ((Vx+Vx0)/2)/f;
		y = est_y;				//y + ((Vy+Vy0)/2)/f;
		z = drone_navdata.altd*0.001;
		Vx0 = Vx;
		Vy0 = Vy;

		r = getDistance(e_x, e_y,e_z, 0,0,0);

		ros::spinOnce(); //if this function is not mentioned the function will stay in the buffer and cannot be able to publish
		loop_rate.sleep();


		}while((ros::ok()));
		myfile.close();

}
void k_hold()
{
	while(ros::ok())
	{

		if(abs(K_est_x)>0.5 || abs(K_est_y)>0.5)
		{
			cout<<"Move"<<K_est_x<<" "<<K_est_y<<endl;
			moveadv(K_est_x,K_est_y,0);
			H =0;
		}
		else
		{
			cout<<"Hold"<<K_est_x<<K_est_y<<endl;
			if(res == 0 && redX > 0 && H==1000)
			{
				int iredX= redX;
				int iredY = redY;
				double Vx = drone_navdata.vx;
				double Vy = drone_navdata.vy;
				H = 0;

				cout<<"Interrupt called"<<redX<<setw(10)<<redY<<endl;
				//hover();
				move(0,0,0,0,0,0);
				//onTarget(redX, redY, Vx, Vy);
				//break;
			}
			else
			{
				cout<<"Hold"<<endl;
				move(0,0,0,0,0,0);
				H = H+1;

			}
			ros::spinOnce();
		}
		ros::spinOnce();

	}
	ros::spin();
/*	ax = 0;
	ay = 0;
	//az = 0;

	//Measuring t : current time
	double t0 = ros::Time::now().toSec(); //ros has a 'Time' function in which the current time can be found by using 'now'.                                         							we need time in sec to compute hence 'tosec'

	double e_x =0, e_y=0, e_z=0, x=0, y=0,z =0,r,r0, i=0,z0, Vx, Vy, Vx0=0, Vy0=0;
	z0 = drone_navdata.altd*0.001;


	r0 = getDistance(0, 0, 0, x_d, y_d, z_d);

	ros::Rate loop_rate(f);

	ofstream myfile;
	myfile.open("/home/icgel/kinect_holds.txt", ios::out | ios::app );
	//myfile<<"Entering the loop";

	z = z0;
	r = r0;

	do{
		double t1=ros::Time::now().toSec();

		//cout<<endl<<"ALT = "<<z<<endl;

		e_x = x - x_d;
		e_y = y - y_d;
		e_z = z - z_d;

		myfile<<endl<<i<<setw(20);

		if(r>0.2)
		{
			Vx = (drone_navdata.vx*0.001);
			Vy = (drone_navdata.vy*0.001);
			lx =  -Kp*0.025*(e_x);
			ly = -Kp*0.025*(e_y);
			lz = 0;
			myfile<<lx<<setw(25);
			myfile<<ly<<setw(25);
			myfile<<lz<<setw(25);
			cout<<"Cmd_vel"<<lx<<setw(10)<<ly<<setw(10)<<lz<<endl;
		}
		else
		{
			lx = 0;
			ly = 0;
			lz = 0;
			cout<<"Else"<<endl;
//			break;
		}

		i = i+1;
		//cout<<x<<setw(20)<<y<<setw(20)<<z<<endl;
//		cout<<"Iteration"<<i<<endl;

		move(lx, ly, lz, ax, ay, az);
		cout<<"co or"<<x<<setw(10)<<y<<setw(10)<<z<<endl;
/*
		myfile<<drone_navdata.header.stamp<<setw(50);

		myfile<<drone_navdata.vx*0.001<<setw(25);
		myfile<<drone_navdata.vy*0.001<<setw(25);
		myfile<<drone_navdata.altd*0.001<<setw(25);
		myfile<<drone_navdata.rotX<<setw(25);
		myfile<<drone_navdata.rotY<<setw(25);
		myfile<<drone_navdata.rotZ<<setw(25);
		myfile<<drone_navdata.ax<<setw(25);
		myfile<<drone_navdata.ay<<setw(25);
		myfile<<drone_navdata.az<<setw(25);
		myfile<<ang.header.stamp<<setw(50);
		myfile<<ang.angular_velocity.x<<setw(25);
		myfile<<ang.angular_velocity.y<<setw(25);
		myfile<<ang.angular_velocity.z<<setw(25);


		x = est_x; 				//x + ((Vx+Vx0)/2)/f;
		y = est_y;				//y + ((Vy+Vy0)/2)/f;
		z = est_z;				//drone_navdata.altd*0.001;
		Vx0 = Vx;
		Vy0 = Vy;

		myfile<<x<<setw(25);
		myfile<<y<<setw(25);
		myfile<<z<<setw(25);

		myfile<<e_x<<setw(25);
		myfile<<e_y<<setw(25);
		myfile<<e_z;

		r = getDistance(e_x, 0, 0, 0,0,0);

		ros::spinOnce(); //if this function is not mentioned the function will stay in the buffer and cannot be able to publish
		loop_rate.sleep();


		}while((ros::ok()));
		myfile.close();
*/
}
