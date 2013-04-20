/*
 * homography.cpp
 *
 *  Created on: Nov 21, 2012
 *      Author: kidson
 */

#include "homography_waitinglist/homography_estimation.h"
#include "util.cpp"

//#include <sensor_msgs/image_encodings.h>
#include "ros/console.h"

#include "opencv2/core/core.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/core/eigen.hpp>
#include <cv.h>

#include <Eigen/Core>
//#include <Eigen/LU>
#include <Eigen/Eigen>
#include <Eigen/SVD>

#include <tf/transform_broadcaster.h>

HomographyEstimation::HomographyEstimation () :
  nh_ ("~"), image_transport_ (nh_), matcher_()
{
//  reconfig_callback_ = boost::bind (&HomographyEstimation::reconfigCallback, this, _1, _2);
//  reconfig_srv_.setCallback (reconfig_callback_);

  template_image_ = cv::Mat (cvLoadImage (default_image_filename.c_str (), CV_LOAD_IMAGE_COLOR));

  subscriber_ = image_transport_.subscribeCamera (default_subscribe_topic, 1,
      &HomographyEstimation::imageCallback, this);

  publisher_ = image_transport_.advertise (default_image_matches_topic, 1);

}

HomographyEstimation::~HomographyEstimation ()
{
  // TODO Auto-generated destructor stub
}


void HomographyEstimation::imageCallback (const sensor_msgs::ImageConstPtr & msg,const sensor_msgs::CameraInfoConstPtr& info_msg)
{
  cv::Mat search_image = convertSensorMsgToCV (msg);
  cv::Mat img_matches;
  std::vector<cv::Point2f> template_points,search_points;
  matcher_.getMatches(template_image_, search_image, img_matches, template_points, search_points);

  cv::Mat homography;

  if(template_points.size() >= 4)
  {
    homography = cv::findHomography( template_points, search_points, CV_RANSAC );
    //homography = getHomographyFromMatches(template_points, search_points);
    drawHomographyOnImage(homography, img_matches);
  }

  publisher_.publish (convertCVToSensorMsg (img_matches));

  //conversion
  Eigen::Matrix3d hom_eigen;
  cv::cv2eigen(homography,hom_eigen);

  boost::array<double, 9> K_copied(info_msg->K);
  Eigen::Map<Eigen::Matrix3d> map(K_copied.elems);
  Eigen::Matrix3d k=map;

  publishTransformationFromHomography(k,hom_eigen);

}

void HomographyEstimation::drawHomographyOnImage(const cv::Mat& homography, cv::Mat& img_matches)
{

  //-- Get the corners from the image_1 ( the object to be "detected" )
  std::vector<cv::Point2f> obj_corners(4);
  obj_corners[0] = cvPoint(0,0); obj_corners[1] = cvPoint(  template_image_.cols, 0 );
  obj_corners[2] = cvPoint( template_image_.cols, template_image_.rows ); obj_corners[3] = cvPoint( 0, template_image_.rows );
  std::vector<cv::Point2f> scene_corners(4);

  cv::perspectiveTransform( obj_corners, scene_corners, homography);

  //-- Draw lines between the corners (the mapped object in the scene - image_2 )
  cv::line( img_matches, scene_corners[0] + cv::Point2f( template_image_.cols, 0), scene_corners[1] + cv::Point2f( template_image_.cols, 0), cv::Scalar(0, 255, 0), 4 );
  cv::line( img_matches, scene_corners[1] + cv::Point2f( template_image_.cols, 0), scene_corners[2] + cv::Point2f( template_image_.cols, 0), cv::Scalar( 0, 255, 0), 4 );
  cv::line( img_matches, scene_corners[2] + cv::Point2f( template_image_.cols, 0), scene_corners[3] + cv::Point2f( template_image_.cols, 0), cv::Scalar( 0, 255, 0), 4 );
  cv::line( img_matches, scene_corners[3] + cv::Point2f( template_image_.cols, 0), scene_corners[0] + cv::Point2f( template_image_.cols, 0), cv::Scalar( 0, 255, 0), 4 );

}

void HomographyEstimation::publishTransformationFromHomography(const Eigen::Matrix3d &intrinsic_mat, const Eigen::Matrix3d &hom_eigen)
{
  Eigen::Matrix3d g = intrinsic_mat.inverse()*hom_eigen;
  double l = sqrt(g.col(0).norm()*g.col(1).norm());

  Eigen::Vector3d R1 = g.col(0)/l;
  Eigen::Vector3d R2 = g.col(1)/l;
  Eigen::Vector3d T = g.col(2)/l;
  Eigen::Vector3d c = R1+R2;
  Eigen::Vector3d p = R1.cross(R2);
  Eigen::Vector3d d = c.cross(p);
  Eigen::Vector3d R1_prime = 1.0/sqrt(2)*(c/c.norm()+d/d.norm());
  Eigen::Vector3d R2_prime = 1.0/sqrt(2)*(c/c.norm()-d/d.norm());
  Eigen::Vector3d R3 = R1_prime.cross(R2_prime);
  Eigen::Matrix3d Rotation;

  Rotation.block<3,1>(0,0)=R1;
  Rotation.block<3,1>(0,1)=R2;
  Rotation.block<3,1>(0,2)=R3;

  ROS_DEBUG_STREAM("R1: "<<R1);
  ROS_DEBUG_STREAM("R2: "<<R2);
  ROS_DEBUG_STREAM("R3: "<<R3);
  ROS_DEBUG_STREAM("Rotation: "<<Rotation);

  //publishing tf
  Eigen::Quaternion<double> quat_rot(Rotation);
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(0,0,0) /*T(0), T(1), T(2)*/ );
  transform.setRotation( tf::Quaternion(quat_rot.x(),quat_rot.y(),quat_rot.z(),quat_rot.w()));
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "something"));
}

cv::Mat HomographyEstimation::getHomographyFromMatches (const std::vector<cv::Point2f>& img1_points, const std::vector<cv::Point2f>& img2_points)
{
  std::vector<cv::Point2f> img1_points_norm, img2_points_norm;
  Eigen::Matrix3f normalize_transform_img1, normalize_transform_img2;
  normalizePoints(img1_points,img1_points_norm, normalize_transform_img1);
  normalizePoints(img2_points,img2_points_norm, normalize_transform_img2);

  Eigen::MatrixXf A(2*img1_points_norm.size(),9);
  for(uint i = 0; i < img1_points_norm.size(); i++)
  {
    int A_row = i*2;
    cv::Point2f a;
    Eigen::Vector3f source_point(img1_points_norm[i].x, img1_points_norm[i].y, 1.0);
    Eigen::Vector3f destination_point(img2_points_norm[i].x, img2_points_norm[i].y, 1.0);

    A.block<1,3>(A_row,3) = -destination_point(2) * source_point.transpose();
    A.block<1,3>(A_row,6) = destination_point(1) * source_point.transpose();
    A.block<1,3>(A_row+1,0) = destination_point(2) * source_point.transpose();
    A.block<1,3>(A_row+1,6) = -destination_point(0) * source_point.transpose();

  }

  //ROS_ERROR_STREAM("A " << A);

  Eigen::JacobiSVD<Eigen::MatrixXf> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
  const Eigen::MatrixXf U = svd.matrixU();
  const Eigen::MatrixXf V = svd.matrixV();
  const Eigen::VectorXf S = svd.singularValues();
  //ROS_ERROR_STREAM("U \n" << U);
  //ROS_INFO_STREAM("V \n" << V);
  //ROS_ERROR_STREAM("S \n" << S);

  Eigen::Matrix3f homography_normalized, homography;
  for(int y=0; y<3;y++)
    for(int x=0; x<3;x++)
      homography_normalized(y,x) = V(y*3+x,8);
  homography = normalize_transform_img2.inverse() * homography_normalized * normalize_transform_img1;

  ROS_INFO_STREAM("homograph normalized matrix \n" << homography_normalized);
  ROS_INFO_STREAM("homograph matrix \n" << homography);

  cv::Mat homography_mat(3,3,CV_32F);
  for(int y=0; y<3;y++)
    for(int x=0; x<3;x++)
      homography_mat.at<float>(y, x) = homography(y,x);

  //ROS_INFO_STREAM("homograph cv mat " << homography_mat);
  return homography_mat;
}

void HomographyEstimation::normalizePoints(const std::vector<cv::Point2f>& points,
                                           std::vector<cv::Point2f>& points_normalized,
                                           Eigen::Matrix3f & transform)
{
  double sum_x=0.0, sum_y=0.0;
  double sum_distances=0.0;

  for(std::vector<cv::Point2f>::const_iterator itr = points.begin(); itr != points.end(); itr++)
  {
    sum_x += itr->x;
    sum_y += itr->y;
  }

  const double avg_sum_x = sum_x/((double)(points.size()));
  const double avg_sum_y = sum_y/((double)(points.size()));

  for(std::vector<cv::Point2f>::const_iterator itr = points.begin(); itr != points.end(); itr++)
  {
    cv::Point2f p(itr->x-avg_sum_x, itr->y-avg_sum_y);
    points_normalized.push_back(p);
    sum_distances += sqrt(p.x*p.x + p.y*p.y);
  }

  const double scale_factor = (sum_distances/(double)(points.size())) / (sqrt(2.0));

  for(std::vector<cv::Point2f>::iterator itr = points_normalized.begin(); itr != points_normalized.end(); itr++)
  {
    itr->x = (itr->x / scale_factor);
    itr->y = (itr->y / scale_factor);
  }

  transform(0,0) = 1;
  transform(0,1) = 0;
  transform(1,0) = 0;
  transform(1,1) = 1;
  transform(2,0) = 0;
  transform(2,1) = 0;

  transform(0,2)= -avg_sum_x;
  transform(1,2)= -avg_sum_y;
  transform(2,2)= scale_factor;
}
