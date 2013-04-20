#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/CvBridge.h>
#include <dynamic_reconfigure/server.h>
#include <imagepub_waitinglist/ImageChangerConfigConfig.h>

int brightness;
double contrast;
image_transport::Publisher pub;

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
	sensor_msgs::ImagePtr image = boost::make_shared<sensor_msgs::Image>();
	image->encoding = "bgr8";
	image->width = msg->width;
	image->height = msg->height;
	image->step = msg->step;
	for (int i=0;i<(int)msg->data.size();i++) {
		image->data.push_back(std::max(std::min(msg->data[i]*contrast+brightness,255.0),0.0));
	}
	pub.publish(image);
}

void callback(imagepub_waitinglist::ImageChangerConfigConfig &config, uint32_t level) {
	brightness = config.brightness;
	contrast = config.contrast;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "image_changer");
	dynamic_reconfigure::Server<imagepub_waitinglist::ImageChangerConfigConfig> srv;
	dynamic_reconfigure::Server<imagepub_waitinglist::ImageChangerConfigConfig>::CallbackType f;
	f = boost::bind(&callback, _1, _2);
	srv.setCallback(f);
	ros::NodeHandle nodeHandle("~");
	nodeHandle.param("brightness",brightness,int(0));
	nodeHandle.param("contrast",contrast,double(1));
	image_transport::ImageTransport it(nodeHandle);
	image_transport::Subscriber sub = it.subscribe("/image_original", 1, imageCallback);
	pub = it.advertise("/image", 1);
	ros::spin();
}
