/*
 * main.cpp
 *
 *  Created on: dec 23, 2012
 *      Author: ross kidson
 */

#include <ros/ros.h>
#include <tf/transform_listener.h>

#include "error_publisher/tf_error.h"
#include "error_publisher/total_error.h"

const static std::string tf_name_1 = "/kitty_stereo/left";
const static std::string tf_name_2 = "/estimate";

int main (int argc, char** argv)
{
  ros::init (argc, argv, "error_publisher");

  ros::NodeHandle nh("~") ;

  tf::TransformListener listener;
  ros::Publisher tf_error_pub = nh.advertise<error_publisher::tf_error>("tf_error", 100);
  ros::Publisher total_error_pub = nh.advertise<error_publisher::total_error>("total_error",100);
  ros::Rate loop_rate (30);
  while (ros::ok())
  {
    tf::StampedTransform transform;
    try
    {
      listener.lookupTransform(tf_name_1, tf_name_2, ros::Time(0), transform);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s",ex.what());
    }

    error_publisher::tf_error error;
    error.x = transform.getOrigin().x();
    error.y = transform.getOrigin().y();
    error.z = transform.getOrigin().z();
    error.axis_x = transform.getRotation().getAxis().getX();
    error.axis_y = transform.getRotation().getAxis().getY();
    error.axis_z = transform.getRotation().getAxis().getZ();
    error.angle = transform.getRotation().getAngle();

    error_publisher::total_error total_error;
    total_error.translation_error = pow(pow(error.x,3)+pow(error.y,3)+pow(error.z,3),((double)1/3) );
    total_error.rotation_error = error.angle;

    tf_error_pub.publish(error);
    total_error_pub.publish(total_error);

    ros::spinOnce ();
    loop_rate.sleep ();
  }
}
