#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/CvBridge.h>
#include <dynamic_reconfigure/server.h>
#include <harris_waitinglist/HarrisConfig.h>

int templateSize;
double threshold;
cv::Mat inputImage;

void harrisCornerDetection() {
	cv::Mat grad_x, grad_y;
	cv::Mat abs_grad_x, abs_grad_y;
	cv::Mat grad;
	cv::Mat R(inputImage.rows,inputImage.cols,CV_64F);
	double maxR = 0;
	Sobel(inputImage,grad_x,CV_16S,1,0,3,1,0,cv::BORDER_DEFAULT);
	Sobel(inputImage,grad_y,CV_16S,0,1,3,1,0,cv::BORDER_DEFAULT);
	convertScaleAbs(grad_x,abs_grad_x);
	convertScaleAbs(grad_y,abs_grad_y);
	addWeighted(abs_grad_x,0.5,abs_grad_y,0.5,0,grad);
	for (int x=0;x<inputImage.cols;x++) {
		for (int y=0;y<inputImage.rows;y++) {
			double A = 0;
			double B = 0;
			double C = 0;
			for (int u=-templateSize/2;u<=templateSize/2;u++) {
				for (int v=-templateSize/2;v<=templateSize/2;v++) {
					int columnIndex = x+u;
					int rowIndex = y+v;
					if (columnIndex<0 || columnIndex>=inputImage.cols) {
						continue;
					}
					if (rowIndex<0 || rowIndex>=inputImage.rows) {
						continue;
					}
					A += pow(abs_grad_x.at<uchar>(rowIndex,columnIndex),2);
					B += abs_grad_x.at<uchar>(rowIndex,columnIndex)*abs_grad_y.at<uchar>(rowIndex,columnIndex);
					C += pow(abs_grad_y.at<uchar>(rowIndex,columnIndex),2);
				}
			}
			double cornerness = A*C-B*B-0.04*(A+C);
			R.at<double>(y,x) = cornerness;
			maxR = std::max(cornerness,maxR);
		}
	}
	cv::Mat annotatedImage;
	cv::cvtColor(inputImage,annotatedImage,CV_GRAY2RGB);
	for (int x=0;x<inputImage.cols;x++) {
		for (int y=0;y<inputImage.rows;y++) {
			if (R.at<double>(y,x) >= maxR*threshold) {
				circle(annotatedImage,cvPoint(x,y),2,CV_RGB(0,0,255));
			}
		}
	}
	imshow("Harris Corner Detection",annotatedImage);
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
	sensor_msgs::CvBridge bridge;
	try {
		cv::Mat inputImageTemp = bridge.imgMsgToCv(msg,"mono8");
		inputImage = inputImageTemp.clone();
		harrisCornerDetection();
	} catch (sensor_msgs::CvBridgeException& e) {
		ROS_ERROR("Could not convert from '%s' to 'mono8'.", msg->encoding.c_str());
	}
}

void callback(harris_waitinglist::HarrisConfig &config, uint32_t level) {
	if (config.templateSize%2 == 0) {
		templateSize = config.templateSize-1;
	} else {
		templateSize = config.templateSize;
	}
	threshold = config.threshold;
	harrisCornerDetection();
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "harris_corners");
	dynamic_reconfigure::Server<harris_waitinglist::HarrisConfig> srv;
	dynamic_reconfigure::Server<harris_waitinglist::HarrisConfig>::CallbackType f;
	f = boost::bind(&callback, _1, _2);
	srv.setCallback(f);
	ros::NodeHandle nodeHandle("~");
	nodeHandle.param("template_size",templateSize,int(3));
	nodeHandle.param("threshold",threshold,double(0.1));
	cvNamedWindow("Harris Corner Detection");
	cvStartWindowThread();
	image_transport::ImageTransport it(nodeHandle);
	image_transport::Subscriber sub = it.subscribe("/image_original", 1, imageCallback);
	ros::spin();
	cvDestroyWindow("Harris Corner Detection");
}
