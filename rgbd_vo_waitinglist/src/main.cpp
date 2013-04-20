/*
 * main.cpp
 *
 *  Created on: Dec 17, 2012
 *      Author: Ross Kidson
 */


#include "rgbd_vo_waitinglist/rgbd_odometry.h"
#include "ros/ros.h"

int main (int argc, char** argv)
{
  ros::init (argc, argv, "rgbd_odometry");
  RGBDOdometry odometry;
  ros::spin();
}



