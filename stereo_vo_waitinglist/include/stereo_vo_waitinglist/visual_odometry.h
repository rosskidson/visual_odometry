/*
 * visual_odometry.h
 *
 *  Created on: Nov 23, 2012
 *      Author: Karol Hausman
 */

#ifndef VISUAL_ODOMETRY_H_
#define VISUAL_ODOMETRY_H_
#include "ros/ros.h"
#include "image_transport/image_transport.h"
#include <image_transport/subscriber_filter.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <Eigen/Core>

#include <sensor_msgs/PointCloud2.h>

#include <ransac_waitinglist/ransac_transformation.h>
#include <feature_cv_waitinglist/feature_matching.h>

#include <tf/transform_broadcaster.h>


typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
		sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> MySyncPolicy;

const static double default_disparity_min_range = 0.0;
const static double default_disparity_max_range = 10000.0;
const static double default_ransac_max_distance_inliers = 1;
const static double default_neighbor_distance_threshold_ = 10;
const static int default_pcl_matches_min_number = 3;
const static int default_ransac_iterations = 30;



class VisualOdometry {

public:
	VisualOdometry();
	virtual ~VisualOdometry();

private:
	void imagesCallback(const sensor_msgs::ImageConstPtr & msg1,
			const sensor_msgs::ImageConstPtr & msg2,
			const sensor_msgs::CameraInfoConstPtr& info_msg_left,
			const sensor_msgs::CameraInfoConstPtr& info_msg_right);

    void drawInfoOnImg(const int inliers, const int inlier_percentage, cv::Mat &image);

	ros::NodeHandle nh_;
	image_transport::ImageTransport image_transport_;
	image_transport::SubscriberFilter subscriber_left_;
	image_transport::SubscriberFilter subscriber_right_;
	message_filters::Subscriber<sensor_msgs::CameraInfo> lcinfo_sub_; /**< Left camera info msg. */
	message_filters::Subscriber<sensor_msgs::CameraInfo> rcinfo_sub_; /**< Right camera info msg. */
    image_transport::Publisher publisher_;
    image_transport::Publisher publisher_depth_;
    ros::Publisher cloud_pub;
	message_filters::Synchronizer<MySyncPolicy> sync_;
	double disparity_min_range_;
	double disparity_max_range_;

  FeatureMatching matcher_;
  RANSACTransformation ransac_transformer_;
  int pcl_matches_min_number_;

  Eigen::Matrix4f final_transformation_, last_transformation_;

  ros::Time end_processing_time_, start_processing_time_, publish_time_;
};

#endif /* VISUAL_ODOMETRY_H_ */
