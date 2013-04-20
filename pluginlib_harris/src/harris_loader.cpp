#include <pluginlib/class_loader.h>
#include <pluginlib_harris/harris_base.h>

int main(int argc, char** argv) {
	pluginlib::ClassLoader<harris_base::CornerDetector> harris_loader("pluginlib_harris", "harris_base::CornerDetector");
	harris_base::CornerDetector* harrisCornerDetector = NULL;
	try {
		harrisCornerDetector = harris_loader.createClassInstance("pluginlib_harris/harris_corner_detector");
		harrisCornerDetector->initialize(3,0.1);
		int result = harrisCornerDetector->findCorners();
		ROS_INFO("result: %d",result);
	}
	catch(pluginlib::PluginlibException& ex) {
		ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
	}
	return 0;
}
