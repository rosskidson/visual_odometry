#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_listener.h>

int main(int argc,char** argv) {
	ros::init(argc,argv,"rviz_chair_logo");
	ros::NodeHandle n;
	ros::Rate rate(10.0);
	ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker",1);
	tf::TransformListener listener;
	while (ros::ok()) {
		tf::StampedTransform transform;
		try {
			listener.lookupTransform("/kitty_stereo/left","/start",ros::Time(0),transform);
		} catch (tf::TransformException ex) {
			ROS_ERROR("TransformException: %s",ex.what());
		}
		visualization_msgs::Marker chairLogo;
		chairLogo.header.frame_id = "/start";
		chairLogo.header.stamp = ros::Time::now();
		chairLogo.ns = "chair_logo";
		chairLogo.id = 0;
		chairLogo.action = visualization_msgs::Marker::ADD;
		chairLogo.type = visualization_msgs::Marker::MESH_RESOURCE;
		chairLogo.mesh_resource = "package://rviz_markers_waitinglist/logo.dae";
		chairLogo.mesh_use_embedded_materials = true;
		chairLogo.lifetime = ros::Duration();
		chairLogo.scale.x = 100.0;
		chairLogo.scale.y = 100.0;
		chairLogo.scale.z = 100.0;
		chairLogo.pose.position.x = transform.getOrigin().x();
		chairLogo.pose.position.y = transform.getOrigin().y();
		chairLogo.pose.position.z = transform.getOrigin().z();
		chairLogo.pose.orientation.x = transform.getRotation().x();
		chairLogo.pose.orientation.y = transform.getRotation().y();
		chairLogo.pose.orientation.z = transform.getRotation().z();
		chairLogo.pose.orientation.w = transform.getRotation().w();
		marker_pub.publish(chairLogo);
		rate.sleep();
	}
}
