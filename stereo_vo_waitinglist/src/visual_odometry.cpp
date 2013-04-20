/*
 * visual_odometry.cpp
 *
 *  Created on: Nov 23, 2012
 *      Author: Karol Hausman
 */

#include "stereo_vo_waitinglist/visual_odometry.h"
#include <ransac_waitinglist/ransac_transformation.h>
#include <feature_cv_waitinglist/feature_matching.h>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <cv.h>
#include <image_geometry/stereo_camera_model.h>
#include "util.cpp"
#include <pcl/common/transforms.h>

const static CvScalar font_color = cvScalar(255,200,200);
const static int font_pos_x = 10;
const static int font_pos_y = 15;
const static int font_row_size = 10;
const static int sync_queue_size = 100;


VisualOdometry::VisualOdometry():
    nh_ ("~"),image_transport_ (nh_),
    subscriber_left_( image_transport_, "/stereo/left/image_rect", 1 ),
    subscriber_right_( image_transport_, "/stereo/right/image_rect", 1 ),
    lcinfo_sub_(nh_,"/stereo/left/camera_info",1),
    rcinfo_sub_(nh_,"/stereo/right/camera_info",1),
    sync_( MySyncPolicy(sync_queue_size), subscriber_left_, subscriber_right_ ,lcinfo_sub_,rcinfo_sub_),
    //sync_( MySyncPolicy(10), subscriber_left_, subscriber_right_ ),
    disparity_min_range_(0),
    disparity_max_range_(100),
    matcher_(),
    ransac_transformer_(),
    pcl_matches_min_number_(3)
{
    cloud_pub = nh_.advertise<sensor_msgs::PointCloud2>("cloud_from_stereo", 1);
    publisher_ = image_transport_.advertise ("stereo_matches", 1);
    publisher_depth_ = image_transport_.advertise ("depth_points_disparity", 1);

    final_transformation_ = Eigen::Matrix4f::Identity();
    last_transformation_  = Eigen::Matrix4f::Identity();

    sync_.registerCallback( boost::bind( &VisualOdometry::imagesCallback, this, _1, _2,_3,_4 ) );
    //sync_.registerCallback( boost::bind( &VisualOdometry::imagesCallback, this, _1, _2 ) );
}

VisualOdometry::~VisualOdometry() {

}

void VisualOdometry::imagesCallback(const sensor_msgs::ImageConstPtr & msg1,
                                    const sensor_msgs::ImageConstPtr & msg2,
                                    const sensor_msgs::CameraInfoConstPtr& info_msg_left,
                                    const sensor_msgs::CameraInfoConstPtr& info_msg_right )
{
    start_processing_time_ = ros::Time::now();
    cv::Mat image_left = convertSensorMsgToCV (msg1);
    cv::Mat image_right = convertSensorMsgToCV (msg2);

    cv::Mat matches_image;
    std::vector<cv::Point2f> left_image_correspondences,right_image_correspondences,prev_left_image_correspondences,prev_right_image_correspondences;
    matcher_.getMatches(image_left,image_right,matches_image,left_image_correspondences,right_image_correspondences,prev_left_image_correspondences,prev_right_image_correspondences);
    publisher_.publish (convertCVToSensorMsg (matches_image));

    float disparity=0;
    float prev_disparity=0;
    pcl::PointCloud<pcl::PointXYZ>::Ptr publish_cloud(
                new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr previous_cloud(
                new pcl::PointCloud<pcl::PointXYZ>);
    sensor_msgs::PointCloud2 cloud_msg;

    cv::Mat image_left_color;
    cv::cvtColor(image_left,image_left_color,CV_GRAY2RGB);

    //visualization values
    float max_depth_one_color = 40;
    float max_depth_sec_color = 100;
    float max_depth_third_color = 900;

    image_geometry::StereoCameraModel cam_model;
     sensor_msgs::CameraInfo left, right;
    right = *info_msg_right;
    left = *info_msg_left;
    right.header.frame_id = "camera";
    left.header.frame_id = "camera";
    cam_model.fromCameraInfo(left,right);
    //cam_model.fromCameraInfo(*info_msg_left,*info_msg_right);

    for(uint i=0; i<prev_left_image_correspondences.size();i++){

        disparity=left_image_correspondences[i].x-right_image_correspondences[i].x;
        prev_disparity=prev_left_image_correspondences[i].x-prev_right_image_correspondences[i].x;


        cv::Point3d point3d_curr;
        cv::Point3d point3d_prev;

        cam_model.projectDisparityTo3d(left_image_correspondences[i], disparity,point3d_curr);
        cam_model.projectDisparityTo3d(prev_left_image_correspondences[i], prev_disparity,point3d_prev);

        pcl::PointXYZ prev_point;
        pcl::PointXYZ point;
        if((point3d_curr.z>=disparity_min_range_)&&(point3d_curr.z<=disparity_max_range_)&&(point3d_prev.z>=disparity_min_range_)&&(point3d_prev.z<=disparity_max_range_))
        {
            point.x=point3d_curr.x;
            point.y=point3d_curr.y;
            point.z=point3d_curr.z;
            prev_point.x=point3d_prev.x;
            prev_point.y=point3d_prev.y;
            prev_point.z=point3d_prev.z;

            publish_cloud->points.push_back(point);
            previous_cloud->points.push_back(prev_point);

            /*visualization on the image*/
            int depth_scaled = static_cast<int>((point3d_curr.z/max_depth_one_color)*255);
            int depth_scaled_2 = static_cast<int>((point3d_curr.z/max_depth_sec_color)*255);
            int depth_scaled_3 = static_cast<int>((point3d_curr.z/max_depth_third_color)*255);

            cv::Scalar color;
            cv::Scalar color_blue(255,255,0);

            if(depth_scaled<=255)
                color=cv::Scalar(depth_scaled,depth_scaled,255);
            else if(depth_scaled_2<=255)
                color=cv::Scalar(255,255-depth_scaled_2,255-depth_scaled_2);
            else
                color=cv::Scalar(255-depth_scaled_3,255,255-depth_scaled_3);

            cv::circle(image_left_color,left_image_correspondences[i],2,color,-1);
            cv::line(image_left_color,prev_left_image_correspondences[i],left_image_correspondences[i],color_blue);
        }

    }

    Eigen::Matrix4f transformation_result=Eigen::Matrix4f::Identity();
    int inliers(0);
    if (previous_cloud->points.size()>=static_cast<uint>(pcl_matches_min_number_)){

        inliers = ransac_transformer_.ransacUmeyama(publish_cloud,previous_cloud,transformation_result);
        ROS_DEBUG_STREAM("transform: \n"<<transformation_result);
    }else{
        transformation_result=last_transformation_; // constant speed assumption
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr publish_cloud_transformed(
                new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*publish_cloud,*publish_cloud_transformed,final_transformation_);

    //update
    last_transformation_ = transformation_result;
    final_transformation_=final_transformation_*transformation_result;
    ROS_DEBUG_STREAM("Final transform: \n"<<final_transformation_);

    //publish tf
    if (previous_cloud->points.size()>=static_cast<uint>(pcl_matches_min_number_)){
        Eigen::Matrix3f Rotation= final_transformation_.block<3,3>(0,0);

        Eigen::Quaternion<float> quat_rot(Rotation);
        static tf::TransformBroadcaster br;
        tf::Transform transform;
        transform.setOrigin( tf::Vector3(final_transformation_(0,3), final_transformation_(1,3), final_transformation_(2,3)) );
        transform.setRotation( tf::Quaternion(quat_rot.x(),quat_rot.y(),quat_rot.z(),quat_rot.w()));
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "start", "estimate"));
    }

    //publish image with depth info
    end_processing_time_ = ros::Time::now();
    const int inliers_percentage = floor(((double)inliers/left_image_correspondences.size())*100);
    drawInfoOnImg(inliers, inliers_percentage, image_left_color);
    publisher_depth_.publish (convertCVToSensorMsg(image_left_color));

    //publish the point cloud
    pcl::toROSMsg(*publish_cloud,cloud_msg);
    cloud_msg.header.stamp=ros::Time::now();
    cloud_msg.header.frame_id="/estimate";
    cloud_msg.header.seq=0;
    cloud_pub.publish(cloud_msg);

}


void VisualOdometry::drawInfoOnImg(const int inliers, const int inlier_percentage, cv::Mat& image)
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
