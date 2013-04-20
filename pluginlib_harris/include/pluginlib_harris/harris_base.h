#ifndef PLUGINLIB_HARRIS_HARRIS_BASE_H_
#define PLUGINLIB_HARRIS_HARRIS_BASE_H_

namespace harris_base {
class CornerDetector {
public:
	virtual void initialize(int _templateSize, double _threshold) = 0;
	virtual int findCorners() = 0;
	virtual ~CornerDetector(){}
protected:
	CornerDetector(){}
};
};
#endif
