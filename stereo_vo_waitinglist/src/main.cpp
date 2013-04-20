/*
 * main.cpp
 *
 *  Created on: Nov 23, 2012
 *      Author: Karol Hausman
 */


#include "stereo_vo_waitinglist/visual_odometry.h"
#include "ros/ros.h"

int main (int argc, char** argv)
{
  ros::init (argc, argv, "visual_odometry");

  VisualOdometry odometry;

  ros::Rate loop_rate (30);
  while (ros::ok())
  {
    ros::spinOnce ();
    loop_rate.sleep ();
  }
}



