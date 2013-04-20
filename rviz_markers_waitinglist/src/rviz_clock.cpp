#define _USE_MATH_DEFINES
#include <math.h>
#include <ctime>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_broadcaster.h>

int main(int argc,char** argv) {
	ros::init(argc,argv,"rviz_clock");
	ros::NodeHandle n;
	ros::Rate r(1);
	ros::Publisher marker_pub = n.advertise<visualization_msgs::MarkerArray>("visualization_marker_array",1);
	uint32_t shape = visualization_msgs::Marker::ARROW;
	tf::Vector3 up_vector(0.0,0.0,-1.0);
	while (ros::ok()) {
		time_t now = time(0);
		tm *ltm = localtime(&now);
		visualization_msgs::MarkerArray markerArray;

		visualization_msgs::Marker marker_sec;
		marker_sec.header.frame_id = "/clock_frame";
		marker_sec.header.stamp = ros::Time::now();
		marker_sec.ns = "clockhands";
		marker_sec.id = 0;
		marker_sec.type = shape;
		marker_sec.action = visualization_msgs::Marker::ADD;

		tf::Quaternion quaternion_sec(up_vector,2.0*M_PI*ltm->tm_sec/60.0);
		quaternion_sec.normalize();
		geometry_msgs::Quaternion orientation_sec;
		tf::quaternionTFToMsg(quaternion_sec,orientation_sec);
		marker_sec.pose.orientation = orientation_sec;

		marker_sec.scale.x = 1.0;
		marker_sec.scale.y = 0.5;
		marker_sec.scale.z = 0.5;
		marker_sec.color.r = 1.0f;
		marker_sec.color.g = 0.0f;
		marker_sec.color.b = 0.0f;
		marker_sec.color.a = 1.0;
		marker_sec.lifetime = ros::Duration();
		markerArray.markers.push_back(marker_sec);

		visualization_msgs::Marker marker_min;
		marker_min.header.frame_id = "/clock_frame";
		marker_min.header.stamp = ros::Time::now();
		marker_min.ns = "clockhands";
		marker_min.id = 1;
		marker_min.type = shape;
		marker_min.action = visualization_msgs::Marker::ADD;

		tf::Quaternion quaternion_min(up_vector,2.0*M_PI*ltm->tm_min/60.0);
		quaternion_min.normalize();
		geometry_msgs::Quaternion orientation_min;
		tf::quaternionTFToMsg(quaternion_min,orientation_min);
		marker_min.pose.orientation = orientation_min;

		marker_min.scale.x = 1.0;
		marker_min.scale.y = 1.0;
		marker_min.scale.z = 1.0;
		marker_min.color.r = 0.0f;
		marker_min.color.g = 1.0f;
		marker_min.color.b = 0.0f;
		marker_min.color.a = 1.0;
		marker_min.lifetime = ros::Duration();
		markerArray.markers.push_back(marker_min);

		visualization_msgs::Marker marker_h;
		marker_h.header.frame_id = "/clock_frame";
		marker_h.header.stamp = ros::Time::now();
		marker_h.ns = "clockhands";
		marker_h.id = 2;
		marker_h.type = shape;
		marker_h.action = visualization_msgs::Marker::ADD;

		tf::Quaternion quaternion_h(up_vector,2.0*M_PI*ltm->tm_hour/12.0);
		quaternion_h.normalize();
		geometry_msgs::Quaternion orientation_h;
		tf::quaternionTFToMsg(quaternion_h,orientation_h);
		marker_h.pose.orientation = orientation_h;

		marker_h.scale.x = 0.5;
		marker_h.scale.y = 1.0;
		marker_h.scale.z = 1.0;
		marker_h.color.r = 0.0f;
		marker_h.color.g = 0.0f;
		marker_h.color.b = 1.0f;
		marker_h.color.a = 1.0;
		marker_h.lifetime = ros::Duration();
		markerArray.markers.push_back(marker_h);

		marker_pub.publish(markerArray);
		r.sleep();
	}
}
