/*
 * TemplateMatchingAndTracking.h
 *
 *  Created on: Nov 21, 2012
 *      Author: kidson
 */

#ifndef HOMOGRAPHY_ESTIMATION_H_
#define HOMOGRAPHY_ESTIMATION_H_

#include "feature_cv_waitinglist/feature_matching.h"

#include "ros/ros.h"
#include "image_transport/image_transport.h"
#include <opencv2/core/core.hpp>

#include <Eigen/Core>

#include <dynamic_reconfigure/server.h>
const static std::string default_image_filename =
    "/home/karol/ros_workspace/rvc_waitinglist/frame0000.jpg";
//"/home/karol/ros_workspace/rvc_waitinglist/feature_cv_waitinglist/frame0000.jpg";
const static std::string default_image_matches_topic = "/image_matches";
//const static std::string default_subscribe_topic = "/image_raw";
const static std::string default_subscribe_topic = "/image_raw";


class HomographyEstimation
{
  public:
    HomographyEstimation ();

    virtual ~HomographyEstimation ();

  private:

    ros::NodeHandle nh_;

    void imageCallback (const sensor_msgs::ImageConstPtr & msg,const sensor_msgs::CameraInfoConstPtr& info_msg);

    cv::Mat getHomographyFromMatches(const std::vector<cv::Point2f>& img1_points, const std::vector<cv::Point2f>& img2_points);

    void normalizePoints(const std::vector<cv::Point2f>& points,
                         std::vector<cv::Point2f>& points_normalized,
                         Eigen::Matrix3f & transform);

    void drawHomographyOnImage(const cv::Mat& homography, cv::Mat& img_matches);

    void publishTransformationFromHomography(const Eigen::Matrix3d &intrinsic_mat, const Eigen::Matrix3d &hom_eigen);

    image_transport::ImageTransport image_transport_;
    image_transport::CameraSubscriber subscriber_;
    image_transport::Publisher publisher_;

    FeatureMatching matcher_;

    cv::Mat template_image_;

};

#endif /* HOMOGRAPHY_ESTIMATION_H_ */
