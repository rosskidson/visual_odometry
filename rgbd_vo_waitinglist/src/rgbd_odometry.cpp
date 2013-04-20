/*
 * rgbd_odometry.cpp
 *
 *  Created on: Dec 17, 2012
 *      Author: Ross Kidson
 */

#include "rgbd_vo_waitinglist/rgbd_odometry.h"

#include <ransac_waitinglist/ransac_transformation.h>
#include <feature_cv_waitinglist/feature_matching.h>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <cv.h>

#include <Eigen/Core>
#include <Eigen/LU>

#include "util.cpp"
#include <pcl/common/transforms.h>
#include <pcl/common/io.h>
#include <pcl/ros/conversions.h>
#include <pcl/io/pcd_io.h>

const static CvScalar font_color = cvScalar(255,200,200);
const static int font_pos_x = 10;
const static int font_pos_y = 15;
const static int font_row_size = 10;

// if the following is false, the color and depth image will be used to create a pointcloud
const static bool subscribe_to_pointcloud = false;
const static int queue_size = 1;
const static double depth_scaling = 1.0;

typedef message_filters::Subscriber<sensor_msgs::Image> image_sub_type;
typedef message_filters::Subscriber<sensor_msgs::CameraInfo> cinfo_sub_type;
typedef message_filters::Subscriber<sensor_msgs::PointCloud2> pc_sub_type;
typedef message_filters::Subscriber<sensor_msgs::PointCloud2> pc_sub_type;

RGBDOdometry::RGBDOdometry():
    nh_ ("~"),
    prev_dense_cloud_ptr_(new pcl::PointCloud<pcl::PointXYZ>),
    matcher_(),
    ransac_transformer_(),
    cam_info_msg_ (new sensor_msgs::CameraInfo),
    pcl_matches_min_number_(3),
    first_callback_(true)
{
    publisher_depth_ = nh_.advertise<sensor_msgs::Image> ("depth_matches", 1);
    cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("cloud_of_matches", queue_size);

    if(subscribe_to_pointcloud)
    {
      cloud_subscriber_ = nh_.subscribe("/camera/depth_registered/points", queue_size, &RGBDOdometry::imagesCallback, this);
    }
    else
    {
      visua_sub_ = new image_sub_type(nh_, "/camera/rgb/image_color", queue_size);
      depth_sub_ = new image_sub_type(nh_, "/camera/depth/image", queue_size);
      cinfo_sub_ = new cinfo_sub_type(nh_, "/camera/rgb/camera_info", queue_size);
//      no_cloud_sync_ = new message_filters::Synchronizer<NoCloudSyncPolicy>(NoCloudSyncPolicy(queue_size),  *visua_sub_, *depth_sub_, *cinfo_sub_);
//      no_cloud_sync_->registerCallback(boost::bind(&RGBDOdometry::noCloudCallback, this, _1, _2, _3));
      no_cloud_sync_nocaminfo_ = new message_filters::Synchronizer<NoCloudSyncPolicyNoCamInfo>(NoCloudSyncPolicyNoCamInfo(queue_size),  *visua_sub_, *depth_sub_);
      no_cloud_sync_nocaminfo_->registerCallback(boost::bind(&RGBDOdometry::noCloudCallback, this, _1, _2));
    }

    cam_info_msg_ = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/camera/rgb/camera_info");

    final_transformation_ = Eigen::Matrix4f::Identity();
    last_transformation_  = Eigen::Matrix4f::Identity();

}

RGBDOdometry::~RGBDOdometry() {

}

cv::Mat RGBDOdometry::restoreCVMatFromPointCloud (pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in_ptr)
{
  cv::Mat restored_image = cv::Mat (cloud_in_ptr->height, cloud_in_ptr->width, CV_8UC3);
  for (uint rows = 0; rows < cloud_in_ptr->height; rows++)
  {
    for (uint cols = 0; cols < cloud_in_ptr->width; ++cols)
    {
      //      restored_image.at<uint8_t>(rows, cols) = cloud_in->at(cols, rows).r;
      restored_image.at<cv::Vec3b> (rows, cols)[0] = cloud_in_ptr->at (cols, rows).b;
      restored_image.at<cv::Vec3b> (rows, cols)[1] = cloud_in_ptr->at (cols, rows).g;
      restored_image.at<cv::Vec3b> (rows, cols)[2] = cloud_in_ptr->at (cols, rows).r;
    }
  }
  return restored_image;
}

void RGBDOdometry::noCloudCallback (const sensor_msgs::ImageConstPtr& visual_img_msg,
                                      const sensor_msgs::ImageConstPtr& depth_img_msg)
                                    //  const sensor_msgs::CameraInfoConstPtr& cam_info_msg)
{
    //Convert images to OpenCV format
    //cv::Mat depth_float_img = cv_bridge::toCvCopy(depth_img_msg)->image;
    cv::Mat visual_img =  cv_bridge::toCvCopy(visual_img_msg)->image;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr = createXYZPointCloud(depth_img_msg,cam_info_msg_);

    getOdometry(visual_img, cloud_ptr);
}

void RGBDOdometry::imagesCallback(const sensor_msgs::PointCloud2Ptr& dense_cloud_msg)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr dense_cloud_color_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr dense_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*dense_cloud_msg,*dense_cloud_color_ptr);
    pcl::copyPointCloud(*dense_cloud_color_ptr, *dense_cloud_ptr);
    cv::Mat current_image = restoreCVMatFromPointCloud(dense_cloud_color_ptr);
    getOdometry(current_image, dense_cloud_ptr);

    //publish the point cloud
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*dense_cloud_color_ptr,cloud_msg);
    cloud_msg.header.stamp=ros::Time::now();
    cloud_msg.header.frame_id="/estimate";
    cloud_msg.header.seq=0;
    cloud_pub_.publish(cloud_msg);
 }

void RGBDOdometry::getOdometry(const cv::Mat current_image, pcl::PointCloud<pcl::PointXYZ>::Ptr dense_cloud_ptr)
{
    start_processing_time_ = ros::Time::now();
    if(first_callback_)
    {
      first_callback_ = false;
      prev_image_ = current_image.clone();
      pcl::copyPointCloud(*dense_cloud_ptr, *prev_dense_cloud_ptr_);
      return;
    }
    cv::Mat matches_image;
    std::vector<cv::Point2f> current_image_features,prev_image_features;
    matcher_.getMatches(current_image,prev_image_,matches_image,current_image_features,prev_image_features);

    // the following (sparse) clouds are clouds containing only matches and their 3d positions
    pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr prev_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);

    for(uint i=0; i < current_image_features.size(); i++)
    {
        current_cloud_ptr->points.push_back(dense_cloud_ptr->at(current_image_features[i].x, current_image_features[i].y));
        prev_cloud_ptr->points.push_back(prev_dense_cloud_ptr_->at(prev_image_features[i].x, prev_image_features[i].y));
    }

    Eigen::Matrix4f transformation_result=Eigen::Matrix4f::Identity();
    int inliers(0);
    if (prev_cloud_ptr->points.size()>=static_cast<uint>(pcl_matches_min_number_)){

        inliers = ransac_transformer_.ransacUmeyama(current_cloud_ptr,prev_cloud_ptr,transformation_result);
        ROS_DEBUG_STREAM("transform: \n"<<transformation_result);
    }else{
        transformation_result=last_transformation_; // constant speed assumption
    }

    //update
    last_transformation_ = transformation_result;
    final_transformation_=final_transformation_*transformation_result;
    ROS_DEBUG_STREAM("Final transform: \n"<<final_transformation_);

    //publish tf
    if (prev_cloud_ptr->points.size()>=static_cast<uint>(pcl_matches_min_number_)){
        Eigen::Matrix3f Rotation= final_transformation_.block<3,3>(0,0);
        Eigen::Quaternion<float> quat_rot(Rotation);
        static tf::TransformBroadcaster br;
        tf::Transform transform;
        transform.setOrigin( tf::Vector3(final_transformation_(0,3), final_transformation_(1,3), final_transformation_(2,3)) );
        transform.setRotation( tf::Quaternion(quat_rot.x(),quat_rot.y(),quat_rot.z(),quat_rot.w()));
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "start", "estimate"));
    }

    //copy cloud and image from current to previous
    pcl::copyPointCloud(*dense_cloud_ptr, *prev_dense_cloud_ptr_);
    prev_image_ = current_image.clone();

    //publish image
    end_processing_time_ = ros::Time::now();
    const int inliers_percentage = floor(((double)inliers/current_image_features.size())*100);
    drawInfoOnImg(inliers, inliers_percentage, matches_image);
    publisher_depth_.publish (convertCVToSensorMsg(matches_image));
}


void RGBDOdometry::drawInfoOnImg(const int inliers, const int inlier_percentage, cv::Mat& image)
{
  double processing_time = (end_processing_time_ - start_processing_time_).toSec()*1000.0;
  ros::Duration time_since_last_callback = ros::Time::now() - publish_time_;
  publish_time_ = ros::Time::now();
  std::stringstream image_text_matches, image_text_hz, image_text_time;
  image_text_matches << "Inliers:" << inliers << " (" << inlier_percentage << "%)";
  image_text_hz << "Hz: " << (1/time_since_last_callback.toSec());
  image_text_time << "Duration: " << processing_time << "ms";
  cv::putText(image, image_text_matches.str(), cvPoint(font_pos_x,font_pos_y+font_row_size*0),
              cv::FONT_HERSHEY_PLAIN, 0.8, font_color, 1, CV_AA);
  cv::putText(image, image_text_hz.str(), cvPoint(font_pos_x,font_pos_y+font_row_size*1),
              cv::FONT_HERSHEY_PLAIN, 0.8, font_color, 1, CV_AA);
  cv::putText(image, image_text_time.str(), cvPoint(font_pos_x,font_pos_y+font_row_size*2),
      cv::FONT_HERSHEY_PLAIN, 0.8, font_color, 1, CV_AA);
}

///\cond
/** Union for easy "conversion" of rgba data */
typedef union
{
  struct /*anonymous*/
  {
    unsigned char Blue;
    unsigned char Green;
    unsigned char Red;
    unsigned char Alpha;
  };
  float float_value;
  long long_value;
} RGBValue;
///\endcond

pcl::PointCloud<pcl::PointXYZRGB>::Ptr RGBDOdometry::createXYZRGBPointCloud (const sensor_msgs::ImageConstPtr& depth_msg,
                                         const sensor_msgs::ImageConstPtr& rgb_msg,
                                         const sensor_msgs::CameraInfoConstPtr& cam_info)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB> );
  cloud_ptr->header.stamp     = depth_msg->header.stamp;
  cloud_ptr->header.frame_id  = rgb_msg->header.frame_id;
  cloud_ptr->is_dense         = true; //single point of view, 2d rasterized

  float cx, cy, fx, fy;//principal point and focal lengths
  unsigned color_step, color_skip;

  cloud_ptr->height = depth_msg->height;
  cloud_ptr->width = depth_msg->width;
  cx = cam_info->K[2]; //(cloud->width >> 1) - 0.5f;
  cy = cam_info->K[5]; //(cloud->height >> 1) - 0.5f;
  fx = 1.0f / cam_info->K[0];
  fy = 1.0f / cam_info->K[4];
  int pixel_data_size = 3;
  char red_idx = 0, green_idx = 1, blue_idx = 2;
  if(rgb_msg->encoding.compare("mono8") == 0) pixel_data_size = 1;
  if(rgb_msg->encoding.compare("bgr8") == 0) { red_idx = 2; blue_idx = 0; }


  ROS_ERROR_COND(pixel_data_size == 0, "Unknown image encoding: %s!", rgb_msg->encoding.c_str());
  color_step = pixel_data_size * rgb_msg->width / cloud_ptr->width;
  color_skip = pixel_data_size * (rgb_msg->height / cloud_ptr->height - 1) * rgb_msg->width;

  cloud_ptr->points.resize (cloud_ptr->height * cloud_ptr->width);

  const float* depth_buffer = reinterpret_cast<const float*>(&depth_msg->data[0]);
  const uint8_t* rgb_buffer = &rgb_msg->data[0];

  // depth_msg already has the desired dimensions, but rgb_msg may be higher res.
  int color_idx = 0, depth_idx = 0;

  pcl::PointCloud<pcl::PointXYZRGB>::iterator pt_iter = cloud_ptr->begin ();
  for (int v = 0; v < (int)cloud_ptr->height; ++v, color_idx += color_skip)
  {
    for (int u = 0; u < (int)cloud_ptr->width; ++u, color_idx += color_step, ++depth_idx, ++pt_iter)
    {
      pcl::PointXYZRGB& pt = *pt_iter;
      float Z = depth_buffer[depth_idx] * depth_scaling;

      // Check for invalid measurements
      if (std::isnan (Z))
      {
        pt.x = pt.y = pt.z = Z;
      }
      else // Fill in XYZ
      {
        pt.x = (u - cx) * Z * fx;
        pt.y = (v - cy) * Z * fy;
        pt.z = Z;
      }

      // Fill in color
      RGBValue color;
      if(pixel_data_size == 3){
        color.Red   = rgb_buffer[color_idx + red_idx];
        color.Green = rgb_buffer[color_idx + green_idx];
        color.Blue  = rgb_buffer[color_idx + blue_idx];
      } else {
        color.Red   = color.Green = color.Blue  = rgb_buffer[color_idx];
      }
      color.Alpha = 0;
      pt.rgb = color.float_value;
    }
  }
  return cloud_ptr;
}


pcl::PointCloud<pcl::PointXYZ>::Ptr RGBDOdometry::createXYZPointCloud (const sensor_msgs::ImageConstPtr& depth_msg,
                                         const sensor_msgs::CameraInfoConstPtr& cam_info)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZ> );
  cloud_ptr->header.stamp     = depth_msg->header.stamp;
  cloud_ptr->header.frame_id  = depth_msg->header.frame_id;
  cloud_ptr->is_dense         = true; //single point of view, 2d rasterized

  float cx, cy, fx, fy;//principal point and focal lengths

  cloud_ptr->height = depth_msg->height;
  cloud_ptr->width = depth_msg->width;
  cx = cam_info->K[2]; //(cloud->width >> 1) - 0.5f;
  cy = cam_info->K[5]; //(cloud->height >> 1) - 0.5f;
  fx = 1.0f / cam_info->K[0];
  fy = 1.0f / cam_info->K[4];

  cloud_ptr->points.resize (cloud_ptr->height * cloud_ptr->width);

  const float* depth_buffer = reinterpret_cast<const float*>(&depth_msg->data[0]);

  int depth_idx = 0;

  pcl::PointCloud<pcl::PointXYZ>::iterator pt_iter = cloud_ptr->begin ();
  for (int v = 0; v < (int)cloud_ptr->height; ++v)
  {
    for (int u = 0; u < (int)cloud_ptr->width; ++u, ++depth_idx, ++pt_iter)
    {
      pcl::PointXYZ& pt = *pt_iter;
      float Z = depth_buffer[depth_idx] * depth_scaling;

      // Check for invalid measurements
      if (std::isnan (Z))
      {
        pt.x = pt.y = pt.z = Z;
      }
      else // Fill in XYZ
      {
        pt.x = (u - cx) * Z * fx;
        pt.y = (v - cy) * Z * fy;
        pt.z = Z;
      }
    }
  }
  return cloud_ptr;
}
