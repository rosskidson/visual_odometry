/*
 * rgbd_odometry.h
 *
 *  Created on: Dec 17, 2012
 *      Author: Ross Kidson
 */

#ifndef RGBD_ODOMETRY_H_
#define RGBD_ODOMETRY_H_
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

//The policy merges kinect messages with approximately equal timestamp into one callback
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
                                                        sensor_msgs::Image,
                                                        sensor_msgs::CameraInfo> NoCloudSyncPolicy;

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
                                                        sensor_msgs::Image> NoCloudSyncPolicyNoCamInfo;

class RGBDOdometry {

public:
    RGBDOdometry();
    virtual ~RGBDOdometry();

private:
    void imagesCallback(const sensor_msgs::PointCloud2Ptr &dense_cloud_msg);

    void noCloudCallback (const sensor_msgs::ImageConstPtr& visual_img_msg,
                          const sensor_msgs::ImageConstPtr& depth_img_msg);//,
//                          const sensor_msgs::CameraInfoConstPtr& cam_info_msg);

    void drawInfoOnImg(const int inliers, const int inlier_percentage, cv::Mat& image);

    cv::Mat restoreCVMatFromPointCloud (pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in_ptr);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr createXYZRGBPointCloud (const sensor_msgs::ImageConstPtr& depth_msg,
                                             const sensor_msgs::ImageConstPtr& rgb_msg,
                                             const sensor_msgs::CameraInfoConstPtr& cam_info);

    pcl::PointCloud<pcl::PointXYZ>::Ptr createXYZPointCloud (const sensor_msgs::ImageConstPtr& depth_msg,
                                             const sensor_msgs::CameraInfoConstPtr &cam_info);

    void getOdometry(const cv::Mat current_image, pcl::PointCloud<pcl::PointXYZ>::Ptr dense_cloud_ptr);

    ros::NodeHandle nh_;
    ros::Subscriber cloud_subscriber_;
    ros::Publisher publisher_depth_;
    ros::Publisher cloud_pub_;

    message_filters::Synchronizer<NoCloudSyncPolicy>* no_cloud_sync_;
    message_filters::Synchronizer<NoCloudSyncPolicyNoCamInfo>* no_cloud_sync_nocaminfo_;
    message_filters::Subscriber<sensor_msgs::Image> *visua_sub_;
    message_filters::Subscriber<sensor_msgs::Image> *depth_sub_;
    message_filters::Subscriber<sensor_msgs::CameraInfo> *cinfo_sub_;

    sensor_msgs::CameraInfoConstPtr cam_info_msg_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr prev_dense_cloud_ptr_;

    FeatureMatching matcher_;
    RANSACTransformation ransac_transformer_;

    cv::Mat prev_image_;
    int pcl_matches_min_number_;
    bool first_callback_;

    Eigen::Matrix4f final_transformation_, last_transformation_;

    ros::Time end_processing_time_, start_processing_time_, publish_time_;
};

#endif /* RGBD_ODOMETRY_H_ */
