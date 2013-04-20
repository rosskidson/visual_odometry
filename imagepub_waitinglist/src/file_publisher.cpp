#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv/cvwimage.h>
#include <opencv/highgui.h>
#include <cv_bridge/CvBridge.h>

std::string file;
int frequency;

int main(int argc, char** argv) {
	ros::init(argc, argv, "file_publisher");
	ros::NodeHandle nodeHandle("~");
	image_transport::ImageTransport it(nodeHandle);
	image_transport::Publisher pub = it.advertise("/image_original", 1);

	nodeHandle.param("file",file,std::string("unknown"));
	nodeHandle.param("frequency",frequency,int(5));

	cv::WImageBuffer3_b image(cvLoadImage(file.c_str(),CV_LOAD_IMAGE_COLOR));
	sensor_msgs::ImagePtr msg = sensor_msgs::CvBridge::cvToImgMsg(image.Ipl(),"bgr8");

	ros::Rate loop_rate(0.5);
	//	while (nodeHandle.ok()) {
	loop_rate.sleep();
	pub.publish(msg);
	ros::spinOnce();
	loop_rate.sleep();
	pub.publish(msg);
	ros::spinOnce();
	//	}
}
