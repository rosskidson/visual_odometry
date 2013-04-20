/*
 * main.cpp
 *
 *  Created on: Nov 21, 2012
 *      Author: kidson
 */

#include "homography_waitinglist/homography_estimation.h"
#include "ros/ros.h"

int main (int argc, char** argv)
{
  ros::init (argc, argv, "homography");

  HomographyEstimation homo;

  ros::Rate loop_rate (30);
  while (ros::ok())
  {
    ros::spinOnce ();
    loop_rate.sleep ();
  }
}
