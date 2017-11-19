/*
 * write.cpp
 *
 *  Created on: 22-Dec-2015
 *      Author: toshiba
 */
#include "ros/ros.h"						//for sync ros
#include "geometry_msgs/Twist.h"			//for command velocity
#include "std_msgs/Empty.h"	 				//For take off and landing
#include "ardrone_autonomy/Navdata.h"    //Accessing ardrone's published data
#include <stdio.h>
#include <termios.h>
#include <sstream>
#include <iostream>
#include <fstream>
using namespace std;

int writeFile();

int main(int argc, char **argv)

{
    writeFile();
}

int writeFile ()
{
  ofstream myfile;
  myfile.open ("example.txt");
  myfile << "Writing this to a file.\n";
  myfile << "Writing this to a file.\n";
  myfile << "Writing this to a file.\n";
  myfile << "Writing this to a file.\n";
  myfile.close();
  return 0;
}


