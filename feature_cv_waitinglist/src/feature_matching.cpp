/*
 * FeatureMatching.cpp
 *
 *  Created on: Nov 21, 2012
 *      Author: kidson
 */

#include "feature_cv_waitinglist/feature_matching.h"
#include "feature_cv_waitinglist/sift_gpu_wrapper.h"

#include "ros/console.h"

#include "opencv2/core/core.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/gpu/gpu.hpp"
//#include <cv.h>

#include <sstream>
#include <limits>

const static int FAST_threshold = 50;
const static int grid_max_number_keypoints = 1000;
const static bool FREAK_normalize_orientation = true;

const static CvScalar font_color = cvScalar(200,200,250);
const static int font_pos_x = 10;
const static int font_pos_y = 15;
const static int font_row_size = 10;

FeatureMatching::FeatureMatching () :
  nh_ ("~/feature_matcher"),
  reconfig_srv_(nh_),
  tracking_threshold_(default_tracking_threshold),
  horizontal_matches_(default_horizontal_matches),
  tracking_matches_(default_tracking_matches),
  min_tracking_matches_(default_min_tracking_matches),
  constructed_(false)
{
  //  enum extractor_matcher  { FLANN, BRUTEFORCE_HAMMING_1, BRUTEFORCE_HAMMING_2, BRUTEFORCE_L1, BRUTEFORCE_L2 };
  //enum feature_detector   { SIFT_detector, SIFTGPU_detector, SURF_detector, FAST_detector, FAST_grid_detector, MSER_detector, ORB_detector, STAR_detector };
  //enum feature_extractor  { SIFT_extractor, SIFTGPU_extractor, SURF_extractor, ORB_extractor, FREAK_extractor, BRIEF_extractor };

  feature_detector_ptrs_.push_back(boost::shared_ptr<cv::FeatureDetector> (new cv::SiftFeatureDetector()));
  feature_detector_ptrs_.push_back(boost::shared_ptr<cv::FeatureDetector> (new cv::SiftFeatureDetector()));
  feature_detector_ptrs_.push_back(boost::shared_ptr<cv::FeatureDetector> (new cv::SurfFeatureDetector()));
  feature_detector_ptrs_.push_back(boost::shared_ptr<cv::FeatureDetector> (new cv::FastFeatureDetector(FAST_threshold)));
  feature_detector_ptrs_.push_back(boost::shared_ptr<cv::FeatureDetector> (new cv::GridAdaptedFeatureDetector(new cv::FastFeatureDetector(),grid_max_number_keypoints)));
  feature_detector_ptrs_.push_back(boost::shared_ptr<cv::FeatureDetector> (new cv::MserFeatureDetector()));
  feature_detector_ptrs_.push_back(boost::shared_ptr<cv::FeatureDetector> (new cv::OrbFeatureDetector()));
  feature_detector_ptrs_.push_back(boost::shared_ptr<cv::FeatureDetector> (new cv::StarFeatureDetector()));

  feature_extractor_ptrs_.push_back(boost::shared_ptr<cv::DescriptorExtractor> (new cv::SiftDescriptorExtractor()));
  feature_extractor_ptrs_.push_back(boost::shared_ptr<cv::DescriptorExtractor> (new cv::SiftDescriptorExtractor()));
  feature_extractor_ptrs_.push_back(boost::shared_ptr<cv::DescriptorExtractor> (new cv::SurfDescriptorExtractor()));
  feature_extractor_ptrs_.push_back(boost::shared_ptr<cv::DescriptorExtractor> (new cv::OrbDescriptorExtractor()));
  feature_extractor_ptrs_.push_back(boost::shared_ptr<cv::DescriptorExtractor> (new cv::FREAK(FREAK_normalize_orientation)));
  feature_extractor_ptrs_.push_back(boost::shared_ptr<cv::DescriptorExtractor> (new cv::BriefDescriptorExtractor()));

  feature_matcher_ptrs_.push_back(boost::shared_ptr<cv::DescriptorMatcher> (new cv::FlannBasedMatcher()));
  feature_matcher_ptrs_.push_back(boost::shared_ptr<cv::DescriptorMatcher> (new cv::BFMatcher(cv::NORM_HAMMING, false)));
  feature_matcher_ptrs_.push_back(boost::shared_ptr<cv::DescriptorMatcher> (new cv::BFMatcher(cv::NORM_HAMMING2, false)));
  feature_matcher_ptrs_.push_back(boost::shared_ptr<cv::DescriptorMatcher> (new cv::BFMatcher(cv::NORM_L1, false)));
  feature_matcher_ptrs_.push_back(boost::shared_ptr<cv::DescriptorMatcher> (new cv::BFMatcher(cv::NORM_L2, false)));

  reconfig_callback_ = boost::bind (&FeatureMatching::reconfigCallback, this, _1, _2);
  reconfig_srv_.setCallback (reconfig_callback_);

  constructed_ = true;
}

FeatureMatching::~FeatureMatching ()
{
  // TODO Auto-generated destructor stub
}

void FeatureMatching::printParameterOptions(cv::Algorithm& cv_algorithm)
{
  ROS_INFO_STREAM("params for " << cv_algorithm.name());
  std::vector<std::string> param_names;
  cv_algorithm.getParams(param_names);
  for(std::vector<std::string>::iterator itr_s = param_names.begin(); itr_s!=param_names.end(); itr_s++)
  {
    switch(cv_algorithm.paramType(*itr_s))
    {
    case 0: //int
      ROS_INFO_STREAM(*itr_s << " (int): " << cv_algorithm.getInt(*itr_s));
      break;
    case 1: //int
      ROS_INFO_STREAM(*itr_s << " (bool): " << cv_algorithm.getBool(*itr_s));
      break;
    case 2: //int
      ROS_INFO_STREAM(*itr_s << " (double): " << cv_algorithm.getDouble(*itr_s));
      break;
    default:
      ROS_INFO_STREAM(*itr_s << " (unknown): ");
    }
  }
}

void FeatureMatching::setParameter(cv::Algorithm& cv_algorithm, std::string param_name, std::string param_value)
{
  int param_type = cv_algorithm.paramType(param_name);
  ROS_INFO_STREAM("setting [" << cv_algorithm.name() << "] Parameter [" << param_name << "] of type [" << param_type << "] to value [" << param_value << "]");
  switch(param_type)
  {
  case 0: //int
    cv_algorithm.set(param_name,atoi(param_value.c_str()));
    break;
  case 1: //bool
    bool val;
    if(param_value == "0" || param_value == "false" || param_value == "FALSE")
      val = true;
    else
      val = false;
    cv_algorithm.set(param_name, val);
    break;
  case 2: //double
    cv_algorithm.set(param_name,atoi(param_value.c_str()));
    break;
  default:
    ROS_WARN("unknown parameter type, value not set");
  }
}

void FeatureMatching::reconfigCallback (feature_cv_waitinglist::FeatureConfig &config,
    uint32_t level)
{
  //  ROS_INFO("Reconfigure Request: %d %f %s %s %d",
  //            config.int_param, config.double_param,
  //            config.str_param.c_str(),
  //            config.bool_param?"True":"False",
  //            config.size);

//  if((config.feature_extractor == SIFTGPU_extractor && config.feature_detector != SIFTGPU_detector) ||
//     (config.feature_extractor != SIFTGPU_extractor && config.feature_detector == SIFTGPU_detector))
//  {
//    config.feature_detector = SIFTGPU_detector;
//    config.feature_extractor = SIFTGPU_extractor;
//  }

  if(config.feature_detector != feature_detector_)
    printParameterOptions(*feature_detector_ptrs_[config.feature_detector]);

  if(config.feature_extractor != feature_extractor_)
  {
    printParameterOptions(*feature_extractor_ptrs_[config.feature_extractor]);
    prev_matches_.clear();    // the tracking uses previous matches.  if there is a mismatch in matches type then the program will crash!
  }

  if((config.feature_detector_parameter_value != feature_detector_parameter_value_) && constructed_)
    setParameter(*feature_detector_ptrs_[config.feature_detector], config.feature_detector_parameter_name, config.feature_detector_parameter_value);

  if((config.feature_extractor_parameter_value != feature_extractor_parameter_value_) && constructed_)
  {
    setParameter(*feature_extractor_ptrs_[config.feature_extractor], config.feature_extractor_parameter_name, config.feature_extractor_parameter_value);
    prev_matches_.clear();    // the tracking uses previous matches.  if there is a mismatch in matches type then the program will crash!
  }

  // binary descriptors need to use bruteforce or else it will crash
  if((config.feature_extractor == ORB_extractor || config.feature_extractor == BRIEF_extractor || config.feature_extractor == FREAK_extractor)
     && (config.descriptor_matcher != BRUTEFORCE_HAMMING_1 && config.descriptor_matcher != BRUTEFORCE_HAMMING_2))
    config.descriptor_matcher = BRUTEFORCE_HAMMING_1;
  if((config.feature_extractor == SURF_extractor || config.feature_extractor == SIFTGPU_extractor || config.feature_extractor == SIFT_extractor)
    && (config.descriptor_matcher == BRUTEFORCE_HAMMING_1 || config.descriptor_matcher == BRUTEFORCE_HAMMING_2))
    config.descriptor_matcher = FLANN;

  feature_detector_ = config.feature_detector;
  feature_extractor_ = config.feature_extractor;
  descriptor_matcher_ = config.descriptor_matcher;
  max_radius_search_dist_ = config.max_radius_search_dist;
  matching_distance_ratio_threshold_ = config.matching_distance_ratio_threshold;
  horizontal_threshold_ = config.horizontal_threshold;
  horizontal_matches_ = config.horizontal_matches;
  tracking_matches_ = config.tracking_matches;
  drawing_template_=config.drawing_template;
  tracking_threshold_ = config.tracking_threshold;
  distance_factor_ = config.distance_factor;
  feature_detector_parameter_value_ = config.feature_detector_parameter_value;
  feature_extractor_parameter_value_ = config.feature_extractor_parameter_value;
  timing_debug_ = config.timing_debug;
  outlier_removal_ = config.outlier_removal;
  retain_all_prev_matches_ = config.retain_all_prev_matches;
  min_tracking_matches_ = config.min_tracking_matches;
  distinct_matches_ = config.distinct_matches;
  double_check_tracking_matches_ = config.double_check_tracking_matches;
  reduce_search_area_ = config.reduce_search_area;
  search_dist_ = config.search_distance;
}


cv::Mat FeatureMatching::convertToBlackWhite(const cv::Mat& input_image)
{
  cv::Mat output_image;
  if (input_image.channels () > 1)
    cvtColor (input_image, output_image, CV_BGR2GRAY);
  else
    output_image = input_image.clone();
  return output_image;
}

void convertDescriptors(const std::vector<float>& float_descriptors, cv::Mat& mat_descriptors)
{
  //create descriptor matrix
  const int size = float_descriptors.size()/128;
  mat_descriptors = cv::Mat(size, 128, CV_32F);
  for (int y = 0; y < size; y++)
  {
    for (int x = 0; x < 128; x++)
      mat_descriptors.at<float>(y, x) = float_descriptors[y * 128 + x];
  }
}

bool FeatureMatching::getMatches (const cv::Mat& template_image, const cv::Mat& search_image,
                                  cv::Mat& matches_overlay,
                                  std::vector<cv::Point2f>& template_match_points,
                                  std::vector<cv::Point2f>& search_match_points)
{
  std::vector<cv::Point2f> prev_template_match_points, prev_search_match_points;
  return getMatches(template_image, search_image, matches_overlay, template_match_points, search_match_points,prev_template_match_points, prev_search_match_points);
}

bool FeatureMatching::getMatches (const cv::Mat& template_image, const cv::Mat& search_image,
                                  cv::Mat& matches_overlay,
                                  std::vector<cv::Point2f>& template_match_points,
                                  std::vector<cv::Point2f>& search_match_points,
                                  std::vector<cv::Point2f>& prev_template_match_points,
                                  std::vector<cv::Point2f>& prev_search_match_points)
{
  start_processing_time_ = ros::Time::now();
  if(timing_debug_)
    ROS_INFO_STREAM("callback started");
  cv::Mat template_bw_image, search_bw_image;
  std::vector<cv::KeyPoint> template_keypoints, search_keypoints;
  cv::Mat template_descriptors, search_descriptors;

  // convert images to black and white
  template_bw_image = convertToBlackWhite(template_image);
  search_bw_image = convertToBlackWhite(search_image);

  //get features

  if(feature_detector_ == SIFTGPU_detector || feature_extractor_ == SIFTGPU_extractor)
  {
    SIFTGPUGetFeatures(template_bw_image, template_keypoints, template_descriptors);
    SIFTGPUGetFeatures(search_bw_image, search_keypoints, search_descriptors);
  }
  else
  {
    detectFeatures (template_bw_image, template_keypoints);
    extractFeatures (template_bw_image, template_keypoints, template_descriptors);
    detectFeatures (search_bw_image, search_keypoints);
    extractFeatures (search_bw_image, search_keypoints, search_descriptors);
  }
  if((search_keypoints.size() == 0) || (template_keypoints.size()==0))
  {
    ROS_WARN_STREAM("not enough keypoints to do matching");
    return false;
  }

  //do matching
  std::vector<cv::DMatch> good_matches;
  std::vector<cv::DMatch> matches;
  std::vector<uint> template_new_to_old_correspondences, search_new_to_old_correspondences;
//  this->findMatches (template_descriptors, search_descriptors, good_matches);
  this->findMatches (template_keypoints,template_descriptors,search_keypoints,search_descriptors,search_bw_image.rows, search_bw_image.cols, good_matches);


  // filter matches
  bool reset_tracking = (int)prev_matches_.size() < min_tracking_matches_;
  if ((horizontal_matches_) && (tracking_matches_) && !reset_tracking)
  {
    std::vector<cv::DMatch> horizontal_matches;
    getHorizontalMatches(template_keypoints, search_keypoints, good_matches, horizontal_matches);
    filterMatchesUsingTrackingBestMatch(template_keypoints, search_keypoints, template_descriptors, search_descriptors, horizontal_matches, matches, template_new_to_old_correspondences, search_new_to_old_correspondences);
    if(timing_debug_)
      ROS_INFO_STREAM("Found " << matches.size() << " matches after applying horizontal filter and tracking matches");
  }
  else if((tracking_matches_) && !reset_tracking)  // only do this when previous matches available
  {
    filterMatchesUsingTrackingBestMatch(template_keypoints, search_keypoints, template_descriptors, search_descriptors, good_matches, matches, template_new_to_old_correspondences, search_new_to_old_correspondences);
    if(timing_debug_)
      ROS_INFO_STREAM("Found " << matches.size() << " matches after applying tracking matches filter");
  }
  else if (horizontal_matches_)
  {
    getHorizontalMatches(template_keypoints, search_keypoints, good_matches, matches);
    if(timing_debug_)
      ROS_INFO_STREAM("Found " << matches.size() << " matches after applying horizontal filter");
  }
  else
  {
    matches = good_matches;
    if(timing_debug_)
      ROS_INFO_STREAM("Found " << matches.size() << " matches after applying standard filter");
  }
  end_processing_time_ = ros::Time::now();


  // Draw matches on the image
  if(search_image.channels() == 1)
  {
    cv::Mat matches_overlay_bw = search_image.clone();
    cv::cvtColor(matches_overlay_bw,matches_overlay,CV_GRAY2RGB);
  }
  else
    matches_overlay = search_image.clone();
  if(tracking_matches_)
  {
    for(uint i = 0; (i < prev_matches_.size()) && (i < matches.size()) && !reset_tracking; i++)
    {
      cv::line(matches_overlay, search_keypoints[matches[i].trainIdx].pt, prev_search_keypoints_[search_new_to_old_correspondences[i]].pt, cv::Scalar(100,255,255));
      cv::circle(matches_overlay, search_keypoints[matches[i].trainIdx].pt, 1, cv::Scalar(0,255,255), /*thickness*/1, /*lineType*/8, /*shift*/0);
    }
  }
  else if(drawing_template_){
      cv::drawMatches(template_image,template_keypoints,search_image,search_keypoints,matches,matches_overlay,cv::Scalar(),cv::Scalar(),std::vector<char> (),cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
  }
  else
  {
    for(uint i = 0; (i < matches.size()); i++)
    {
      cv::circle(matches_overlay, search_keypoints[matches[i].trainIdx].pt, 1, cv::Scalar(0,255,255), /*thickness*/1, /*lineType*/8, /*shift*/0);
      cv::line(matches_overlay, template_keypoints[matches[i].queryIdx].pt, search_keypoints[matches[i].trainIdx].pt, cv::Scalar(100,255,255));
    }
  }
//  // To display just keypoints:
//  for(uint i = 0; (i < search_keypoints.size()); i++)
//    cv::circle(matches_overlay, search_keypoints[i].pt, 1, cv::Scalar(0,255,255), /*thickness*/1, /*lineType*/8, /*shift*/0);

  //drawInfoOnImg(matches.size(), matches_overlay);
  if(timing_debug_)
    ROS_INFO_STREAM("drew output image");

  //copy matches into 2 vectors
  for(std::vector<cv::DMatch>::const_iterator itr=matches.begin(); itr != matches.end(); itr++)
  {
    template_match_points.push_back(template_keypoints[itr->queryIdx].pt);
    search_match_points.push_back(search_keypoints[itr->trainIdx].pt);
    if(template_new_to_old_correspondences.size() > 0)
    {
      prev_template_match_points.push_back(prev_template_keypoints_[template_new_to_old_correspondences[itr-matches.begin()]].pt);
      prev_search_match_points.push_back(prev_search_keypoints_[search_new_to_old_correspondences[itr-matches.begin()]].pt);
    }
  }

  // save current frame variables to last frame
  prev_template_keypoints_ = template_keypoints;
  prev_search_keypoints_ = search_keypoints;
  prev_template_descriptors_ = template_descriptors.clone();
  prev_search_descriptors_ = search_descriptors.clone();
  if(retain_all_prev_matches_)
    prev_matches_ = good_matches;
  else
    prev_matches_ = matches;
  if(timing_debug_)
    ROS_INFO_STREAM("convert output data and copy results to class variables");
  return true;
}

void FeatureMatching::drawInfoOnImg(const int matches, cv::Mat& image)
{
  double processing_time = (end_processing_time_ - start_processing_time_).toSec()*1000.0;
  if(processing_time > 100.0)
    ROS_ERROR_STREAM("matching too slow for 10Hz. Process time " << processing_time<< "ms");
  ros::Duration time_since_last_callback = ros::Time::now() - publish_time_;
  publish_time_ = ros::Time::now();
  std::stringstream image_text_matches, image_text_hz, image_text_time;
  image_text_matches << "Matches:" << matches;
  image_text_hz << "Hz: " << (1/time_since_last_callback.toSec());
  image_text_time << "Duration: " << processing_time << "ms";
  cv::putText(image, image_text_matches.str(), cvPoint(font_pos_x,font_pos_y+font_row_size*0),
              cv::FONT_HERSHEY_PLAIN, 0.8, font_color, 1, CV_AA);
  cv::putText(image, image_text_hz.str(), cvPoint(font_pos_x,font_pos_y+font_row_size*1),
              cv::FONT_HERSHEY_PLAIN, 0.8, font_color, 1, CV_AA);
  cv::putText(image, image_text_time.str(), cvPoint(font_pos_x,font_pos_y+font_row_size*2),
      cv::FONT_HERSHEY_PLAIN, 0.8, font_color, 1, CV_AA);
}

void FeatureMatching::filterMatches (std::vector<std::vector<cv::DMatch> >& all_matches, std::vector<
    cv::DMatch>& good_matches)
{
  for (uint i = 0; i < all_matches.size(); i++)
    if(all_matches[i].size()>1)
      if(all_matches[i][0].distance / all_matches[i][1].distance < matching_distance_ratio_threshold_)
        good_matches.push_back(all_matches[i][0]);
}

void FeatureMatching::getHorizontalMatches (const std::vector<cv::KeyPoint>& template_keypoints,
                                            const std::vector<cv::KeyPoint>& search_keypoints,
                                            const std::vector<cv::DMatch>& matches,
                                            std::vector<cv::DMatch>& horizontal_matches)
{
  for( uint i = 0; i < matches.size(); i++ )
    if(fabs(template_keypoints[matches[i].queryIdx].pt.y-search_keypoints[matches[i].trainIdx].pt.y)<=horizontal_threshold_)
      horizontal_matches.push_back(matches[i]);
}

double euclideanDist(const cv::Point2f& p, const cv::Point2f& q)
{
    cv::Point2f diff = p - q;
    return cv::sqrt(diff.x*diff.x + diff.y*diff.y);
}

void FeatureMatching::filterMatchesUsingTracking (const std::vector<cv::KeyPoint>& template_keypoints,
                                            const std::vector<cv::KeyPoint>& search_keypoints,
                                            const std::vector<cv::DMatch>& matches,
                                            std::vector<cv::DMatch>& good_matches,
                                            std::vector<uint>& template_new_to_prev_correspondence,
                                            std::vector<uint>& search_new_to_prev_correspondence)
{
  for(std::vector<cv::DMatch>::const_iterator itr_t1=matches.begin(); itr_t1 != matches.end(); itr_t1++)
  {
    const cv::Point2f& template_pt = template_keypoints[itr_t1->queryIdx].pt;
    const cv::Point2f& search_pt = search_keypoints[itr_t1->trainIdx].pt;
    // find the point in the previous frame
    for(std::vector<cv::DMatch>::const_iterator itr_t0=prev_matches_.begin(); itr_t0 != prev_matches_.end(); itr_t0++)
    {
      const cv::Point2f& prev_template_pt = prev_template_keypoints_[itr_t0->queryIdx].pt;
      const cv::Point2f& prev_search_pt = prev_search_keypoints_[itr_t0->trainIdx].pt;
      if(euclideanDist(template_pt, prev_template_pt) < tracking_threshold_)
      {
        if(euclideanDist(search_pt, prev_search_pt) < tracking_threshold_)
        {
           good_matches.push_back(*itr_t1);
           template_new_to_prev_correspondence.push_back(itr_t0->queryIdx);
           search_new_to_prev_correspondence.push_back(itr_t0->trainIdx);
           itr_t0 = prev_matches_.end()-1;
        }
      }
    }
  }
}

void FeatureMatching::filterMatchesUsingTrackingNN (const std::vector<cv::KeyPoint>& template_keypoints,
                                            const std::vector<cv::KeyPoint>& search_keypoints,
                                            const std::vector<cv::DMatch>& matches,
                                            std::vector<cv::DMatch>& good_matches,
                                            std::vector<uint>& template_new_to_prev_correspondence,
                                            std::vector<uint>& search_new_to_prev_correspondence)
{
  std::vector<double> min_point_distance_vec;
  for(std::vector<cv::DMatch>::const_iterator itr_t1=matches.begin(); itr_t1 != matches.end(); itr_t1++)
  {
    const cv::Point2f& template_pt = template_keypoints[itr_t1->queryIdx].pt;
    const cv::Point2f& search_pt = search_keypoints[itr_t1->trainIdx].pt;

    // find the point in the previous frame
    double min_point_distance = std::numeric_limits<double>::max();
    std::vector<cv::DMatch>::const_iterator best_match_itr;
    for(std::vector<cv::DMatch>::const_iterator itr_t0=prev_matches_.begin(); itr_t0 != prev_matches_.end(); itr_t0++)
    {
      const cv::Point2f& prev_template_pt = prev_template_keypoints_[itr_t0->queryIdx].pt;
      const cv::Point2f& prev_search_pt = prev_search_keypoints_[itr_t0->trainIdx].pt;

      const double template_point_distance = euclideanDist(template_pt, prev_template_pt);
      if(template_point_distance > tracking_threshold_) continue;
      const double search_point_distance = euclideanDist(search_pt, prev_search_pt);
      if(search_point_distance > tracking_threshold_) continue;

      const double new_min_point_distance = template_point_distance + search_point_distance;
      if(new_min_point_distance < min_point_distance)
      {
        min_point_distance = new_min_point_distance;
        best_match_itr = itr_t0;
      }
    }

    if(min_point_distance < std::numeric_limits<double>::max())
    {
        good_matches.push_back(*itr_t1);
        min_point_distance_vec.push_back(min_point_distance);
        template_new_to_prev_correspondence.push_back(best_match_itr->queryIdx);
        search_new_to_prev_correspondence.push_back(best_match_itr->trainIdx);
    }
  }

  if(outlier_removal_)
    removeOutliers(good_matches, min_point_distance_vec, template_new_to_prev_correspondence, search_new_to_prev_correspondence);
}

void FeatureMatching::filterMatchesUsingTrackingBestMatch (const std::vector<cv::KeyPoint>& template_keypoints,
                                            const std::vector<cv::KeyPoint>& search_keypoints,
                                            const cv::Mat& template_descriptors,
                                            const cv::Mat& search_descriptors,
                                            const std::vector<cv::DMatch>& matches,
                                            std::vector<cv::DMatch>& good_matches,
                                            std::vector<uint>& template_new_to_prev_correspondence,
                                            std::vector<uint>& search_new_to_prev_correspondence)
{
  std::vector<double> distances;
  for(std::vector<cv::DMatch>::const_iterator itr_t1=matches.begin(); itr_t1 != matches.end(); itr_t1++)
  {
    //std::vector<cv::KeyPoint> prev_search_keypoints, prev_template_keypoints;
    std::vector<int> prev_template_descriptor_mapping, prev_search_descriptor_mapping;
    cv::Mat prev_local_search_descriptors, prev_local_template_descriptors;

    const cv::Point2f& template_pt = template_keypoints[itr_t1->queryIdx].pt;
    const cv::Point2f& search_pt = search_keypoints[itr_t1->trainIdx].pt;

    // find the point in the previous frame
    std::vector<double> local_distances;
    for(std::vector<cv::DMatch>::const_iterator itr_t0=prev_matches_.begin(); itr_t0 != prev_matches_.end(); itr_t0++)
    {
      const cv::Point2f& prev_template_pt = prev_template_keypoints_[itr_t0->queryIdx].pt;
      const cv::Point2f& prev_search_pt = prev_search_keypoints_[itr_t0->trainIdx].pt;

      const double template_point_distance = euclideanDist(template_pt, prev_template_pt);
      if(template_point_distance > tracking_threshold_) continue;
      const double search_point_distance = euclideanDist(search_pt, prev_search_pt);
      if(search_point_distance > tracking_threshold_) continue;

      // store prev_template_descript and prev_search_descript at these indices to a new mat
      prev_local_template_descriptors.push_back(prev_template_descriptors_.row(itr_t0->queryIdx));
      prev_local_search_descriptors.push_back(prev_search_descriptors_.row(itr_t0->trainIdx));
      // remember the mapping between new (prev_local_descriptors) mat and prev_descriptors_
      prev_template_descriptor_mapping.push_back(itr_t0->queryIdx);
      prev_search_descriptor_mapping.push_back(itr_t0->trainIdx);

      local_distances.push_back(template_point_distance + search_point_distance);
    }

    if(prev_local_template_descriptors.rows > 0)
    {
      std::vector<cv::DMatch> point_template_matches, point_search_matches;
      feature_matcher_ptrs_[descriptor_matcher_]->match (template_descriptors.row(itr_t1->queryIdx), prev_local_template_descriptors, point_template_matches);
      feature_matcher_ptrs_[descriptor_matcher_]->match (search_descriptors.row(itr_t1->trainIdx), prev_local_search_descriptors, point_search_matches);

      //point_*_matches[0]->trainIdx -> is the idx of the match in the new temp mat (prev_local_*_descriptors).
      //remap back to prev_template_descriptors_ and add into matches etc
      // use both matches to double check.  only accept when all matches agree
      if(point_template_matches[0].trainIdx == point_search_matches[0].trainIdx || !double_check_tracking_matches_)
      {
        good_matches.push_back(*itr_t1);
        template_new_to_prev_correspondence.push_back(prev_template_descriptor_mapping[point_template_matches[0].trainIdx]);
        search_new_to_prev_correspondence.push_back(prev_search_descriptor_mapping[point_template_matches[0].trainIdx]);
        distances.push_back(local_distances[point_template_matches[0].trainIdx]);
      }
    }
  }
  if(outlier_removal_)
    removeOutliers(good_matches, distances, template_new_to_prev_correspondence, search_new_to_prev_correspondence);
}

void FeatureMatching::removeOutliers(std::vector<cv::DMatch>& matches, std::vector<double>& distances,
                    std::vector<uint>& template_new_to_prev_correspondence,
                    std::vector<uint>& search_new_to_prev_correspondence)
{
  double avg_dist=0.0;
  for(std::vector<double>::iterator itr = distances.begin(); itr < distances.end(); itr++)
      avg_dist += *itr;
  avg_dist /= distances.size();
  for(uint i=0; i<distances.size(); i++)
  {
    if(distances[i] > distance_factor_ * avg_dist)
    {
      matches.erase(matches.begin()+i);
      distances.erase(distances.begin()+i);
      template_new_to_prev_correspondence.erase(template_new_to_prev_correspondence.begin()+i);
      search_new_to_prev_correspondence.erase(search_new_to_prev_correspondence.begin()+i);
      i--;
    }
  }
}

void FeatureMatching::detectFeatures (const cv::Mat& input_image, std::vector<
    cv::KeyPoint>& keypoints)
{
  feature_detector_ptrs_[feature_detector_]->detect (input_image, keypoints);
  if(timing_debug_)
    ROS_INFO_STREAM("Found " << keypoints.size() << " keypoints");
}

void FeatureMatching::extractFeatures (const cv::Mat& input_image, std::vector<
    cv::KeyPoint>& keypoints, cv::Mat& descriptors)
{
  feature_extractor_ptrs_[feature_extractor_]->compute (input_image, keypoints, descriptors);
  if(timing_debug_)
      ROS_INFO_STREAM("Extracted " << keypoints.size() << " keypoints and " << descriptors.rows << " descriptors");
}

void FeatureMatching::SIFTGPUGetFeatures(const cv::Mat& input_image,
      std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors)
{
#ifdef USE_SIFT_GPU
  std::vector<float> descriptors_float;
  SiftGPUWrapper* SIFT_GPU_ptr = SiftGPUWrapper::getInstance();
  SIFT_GPU_ptr->detect(input_image,  keypoints, descriptors_float);
  convertDescriptors(descriptors_float, descriptors);
#endif
}

void FeatureMatching::findMatches (const std::vector<cv::KeyPoint>& source_keypoints,
                                   const cv::Mat& source_descriptors,
                                   const std::vector<cv::KeyPoint>& target_keypoints,
                                   const cv::Mat& target_descriptors,
                                   const int image_height,
                                   const int image_width,
                                   std::vector<cv::DMatch >& matches)
{
  if(reduce_search_area_)
    {
      const int y_search_dist = horizontal_matches_ ? horizontal_threshold_ : search_dist_;
      for(std::vector<cv::KeyPoint>::const_iterator itr_source=source_keypoints.begin(); itr_source!= source_keypoints.end(); itr_source++)
      {
        const int x_l = (itr_source->pt.x - search_dist_ < 0 ? 0 : itr_source->pt.x - search_dist_);
        const int x_u = (itr_source->pt.x + search_dist_ > image_width ? image_width : itr_source->pt.x + search_dist_);
        const int y_l = (itr_source->pt.y - y_search_dist < 0 ? 0 : itr_source->pt.y - y_search_dist);
        const int y_u = (itr_source->pt.y + y_search_dist > image_height ? image_height : itr_source->pt.y + y_search_dist);
        //ROS_INFO_STREAM("xl " << x_l << " xu " << x_u );
        //ROS_INFO_STREAM("yl " << y_l << " yu " << y_u );
        cv::Mat nearby_descriptors;
        std::vector<int> nearby_indices;
        for(std::vector<cv::KeyPoint>::const_iterator itr_target=target_keypoints.begin(); itr_target!=target_keypoints.end(); itr_target++)
          if( (itr_target->pt.x >= x_l) && (itr_target->pt.x <= x_u) && (itr_target->pt.y >= y_l) && (itr_target->pt.y <= y_u) )
          {
            nearby_descriptors.push_back(target_descriptors.row(itr_target-target_keypoints.begin()));
            nearby_indices.push_back(itr_target-target_keypoints.begin());
          }

        if(nearby_descriptors.rows > 0)
        {
          std::vector<cv::DMatch > single_match;
          feature_matcher_ptrs_[descriptor_matcher_]->match (source_descriptors.row(itr_source-source_keypoints.begin()), nearby_descriptors, single_match);
          single_match[0].queryIdx = itr_source - source_keypoints.begin();
          single_match[0].trainIdx = nearby_indices[single_match[0].trainIdx];
          if(distinct_matches_)
          {
            if(single_match[0].distance < max_radius_search_dist_)
              matches.push_back(single_match[0]);
          }
          else
            matches.push_back(single_match[0]);
        }
      }
    }
  else if(distinct_matches_)
  {
    std::vector<std::vector<cv::DMatch> > initial_matches;
    feature_matcher_ptrs_[descriptor_matcher_]->radiusMatch (source_descriptors, target_descriptors, initial_matches, (float)max_radius_search_dist_);
    filterMatches (initial_matches, matches);
  }
  else
    feature_matcher_ptrs_[descriptor_matcher_]->match (source_descriptors, target_descriptors, matches);

  if(timing_debug_)
    ROS_INFO_STREAM("Found " << matches.size() << " matches");
}


void FeatureMatching::getDescriptorsInBox(const int x_l, const int x_u, const int y_l, const int y_u,
                         const std::vector<cv::KeyPoint>& target_keypoints,
                         const cv::Mat& target_descriptors,
                         cv::Mat& nearby_descriptors)
{

}
