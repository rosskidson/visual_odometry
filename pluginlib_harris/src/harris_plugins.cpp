#include <pluginlib/class_list_macros.h>
#include <pluginlib_harris/harris_base.h>
#include <pluginlib_harris/harris_plugins.h>

PLUGINLIB_DECLARE_CLASS(pluginlib_harris,harris_corner_detector,harris_plugins::HarrisCornerDetector,harris_base::CornerDetector)
