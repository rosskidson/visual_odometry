/*
 * main.cpp
 *
 *  Created on: dec 4, 2012
 *      Author: ross kidson
 */


#include "feature_cv_waitinglist/feature_matching.h"

#include "ros/ros.h"

#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>


#include "util.cpp"

#include <string>

const static std::string template_filename = "/home/ross/frame0000.jpg";///work/kidson/frame0000.jpg";
const static std::string subscribe_topic ="/image_raw/"; // /camera/rgb/image_color";
const static std::string image_matches_topic = "/image_matches";
const static std::string left_image_topic = "/kitty_stereo/left/image_rect";
const static std::string right_image_topic = "/kitty_stereo/right/image_rect";
const static std::string left_camera_info_topic = "/kitty_stereo/left/camera_info";
const static std::string right_camera_info_topic = "/kitty_stereo/right/camera_info";


class MatcherTestbed
{
  FeatureMatching matcher_;
  cv::Mat template_image_;
  bool first_callback_;

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
      sensor_msgs::Image> MySyncPolicy;
  image_transport::Publisher publisher_;
  image_transport::Subscriber subscriber_;
  image_transport::ImageTransport image_transport_;
  image_transport::SubscriberFilter subscriber_left_;
  image_transport::SubscriberFilter subscriber_right_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> lcinfo_sub_; /**< Left camera info msg. */
  message_filters::Subscriber<sensor_msgs::CameraInfo> rcinfo_sub_; /**< Right camera info msg. */
  message_filters::Synchronizer<MySyncPolicy> sync_;

public:
  MatcherTestbed(ros::NodeHandle nh):
    matcher_(), image_transport_(nh),
    subscriber_left_( image_transport_, left_image_topic, 1 ),
    subscriber_right_( image_transport_, right_image_topic, 1 ),
      lcinfo_sub_(nh,left_camera_info_topic,1),
      rcinfo_sub_(nh,right_camera_info_topic,1),
      sync_( MySyncPolicy(10), subscriber_left_, subscriber_right_ ), first_callback_(false)
  {
    template_image_ = cv::Mat (cvLoadImage (template_filename.c_str (), CV_LOAD_IMAGE_COLOR));
    subscriber_ = image_transport_.subscribe(subscribe_topic, 1, &MatcherTestbed::imageCallback, this);
    sync_.registerCallback( boost::bind( &MatcherTestbed::stereoImagesCallback, this, _1, _2) );

    publisher_ = image_transport_.advertise (image_matches_topic, 1);
  }

  void imageCallback (const sensor_msgs::ImageConstPtr & msg)
  {
    if(!first_callback_)
    {
      cv::Mat search_image = convertSensorMsgToCV (msg);
      cv::Mat img_matches;
      std::vector<cv::Point2f> template_points,search_points;

      matcher_.getMatches(template_image_, search_image, img_matches, template_points, search_points);
      publisher_.publish (convertCVToSensorMsg (img_matches));
    }
    template_image_ = convertSensorMsgToCV (msg);
  }


  void stereoImagesCallback(const sensor_msgs::ImageConstPtr & msg1,
      const sensor_msgs::ImageConstPtr & msg2 )
  {
    cv::Mat img_matches;
    cv::Mat image_left = convertSensorMsgToCV (msg1);
    cv::Mat image_right = convertSensorMsgToCV (msg2);
    std::vector<cv::Point2f> template_points,search_points;

    matcher_.getMatches(image_left, image_right, img_matches, template_points, search_points);

    publisher_.publish (convertCVToSensorMsg (img_matches));
  }
};
int main (int argc, char** argv)
{
  ros::init (argc, argv, "matches_testbed");

  ros::NodeHandle nh("~") ;
  MatcherTestbed matcher(nh);

  ros::Rate loop_rate (30);
  while (ros::ok())
  {
    ros::spinOnce ();
    loop_rate.sleep ();
  }
}
