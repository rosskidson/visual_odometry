#ifndef PLUGINLIB_HARRIS_HARRIS_PLUGINS_H_
#define PLUGINLIB_HARRIS_HARRIS_PLUGINS_H_
#include <pluginlib_harris/harris_base.h>

namespace harris_plugins {
	class HarrisCornerDetector : public harris_base::CornerDetector {
	public:
		HarrisCornerDetector(){}
		void initialize(int _templateSize, double _threshold) {
			templateSize = _templateSize;
			threshold = _threshold;
		}
		int findCorners() {
			return 1337;
		}
	private:
		int templateSize;
		double threshold;
	};
};
#endif
