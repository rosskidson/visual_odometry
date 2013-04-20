#include <ros/ros.h>
#include <image_transport/image_transport.h>

int width;
int height;
int square_size;
int frequency;

int main(int argc, char** argv) {
	ros::init(argc, argv, "image_publisher");
	ros::NodeHandle nodeHandle("~");

	nodeHandle.param("width",width,int(500));
	nodeHandle.param("height",height,int(500));
	nodeHandle.param("square_size",square_size,int(100));
	nodeHandle.param("frequency",frequency,int(5));

	image_transport::ImageTransport imageTransport(nodeHandle);
	image_transport::Publisher pub = imageTransport.advertise("/image_checkboard",1);

	sensor_msgs::ImagePtr image = boost::make_shared<sensor_msgs::Image>();
	bool paintingWhite = false;
	for (int i=0;i<width*height;i++) {
		if (i%square_size == 0) {
			paintingWhite = !paintingWhite;
		}
		image->data.push_back(paintingWhite?255:0);
	}
	image->width = width;
	image->height = height;
	image->step = width;
	image->encoding = "mono8";
	image->header.seq = 1;
	image->header.frame_id	= 1;
	image->header.stamp = ros::Time::now();

	ros::Rate loop_rate(frequency);
	while (nodeHandle.ok()) {
		pub.publish(image);
		ros::spinOnce();
		loop_rate.sleep();
	}
}
