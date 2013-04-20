/*
 * FeatureMatching.h
 *
 *  Created on: Nov 21, 2012
 *      Author: kidson
 */

#ifndef FEATUREMATCHING_H_
#define FEATUREMATCHING_H_

#include "ros/ros.h"
#include <opencv2/core/core.hpp>
#include <opencv2/nonfree/features2d.hpp>

#include <dynamic_reconfigure/server.h>
#include "../../cfg/cpp/feature_cv_waitinglist/FeatureConfig.h"

const static std::string default_feature = "SURF";
const static std::string default_descriptor_matcher = "FLANN";
const static double default_max_radius_search_dist = 0.2;
const static double default_matching_distance_ratio_threshold = 0.8;
const static bool default_horizontal_matches=false;
const static int default_horizontal_threshold=5;
const static bool default_tracking_matches=true;
const static double default_tracking_threshold=0.1;
const static int default_min_tracking_matches=5;

class FeatureMatching
{

  enum extractor_matcher  { FLANN, BRUTEFORCE_HAMMING_1, BRUTEFORCE_HAMMING_2, BRUTEFORCE_L1, BRUTEFORCE_L2 };
  enum feature_detector   { SIFT_detector, SIFTGPU_detector, SURF_detector, FAST_detector, FAST_grid_detector, MSER_detector, ORB_detector, STAR_detector };
  enum feature_extractor  { SIFT_extractor, SIFTGPU_extractor, SURF_extractor, ORB_extractor, FREAK_extractor, BRIEF_extractor };

  public:
    FeatureMatching ();

    virtual ~FeatureMatching ();

    bool getMatches (const cv::Mat& template_image, const cv::Mat& search_image,
                                      cv::Mat& matches_overlay,
                                      std::vector<cv::Point2f>& template_match_points,
                                      std::vector<cv::Point2f>& search_match_points);

    bool getMatches (const cv::Mat& template_image, const cv::Mat& search_image,
                                      cv::Mat& matches_overlay,
                                      std::vector<cv::Point2f>& template_match_points,
                                      std::vector<cv::Point2f>& search_match_points,
                                      std::vector<cv::Point2f>& prev_template_match_points,
                                      std::vector<cv::Point2f>& prev_search_match_points);
  private:

    cv::Mat convertToBlackWhite(const cv::Mat& input_image);

    void detectFeatures (const cv::Mat& input_image, std::vector<
        cv::KeyPoint>& keypoints);

    void extractFeatures (const cv::Mat& input_image, std::vector<
        cv::KeyPoint>& keypoints, cv::Mat& descriptors);

    void SIFTGPUGetFeatures(const cv::Mat& input_image,
          std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors);

    void findMatches  (const std::vector<cv::KeyPoint>& source_keypoints,
                       const cv::Mat& source_descriptors,
                       const std::vector<cv::KeyPoint>& target_keypoints,
                       const cv::Mat& target_descriptors,
                       const int image_height,
                       const int image_width,
                       std::vector<cv::DMatch >& matches);


    void reconfigCallback (feature_cv_waitinglist::FeatureConfig &config, uint32_t level);

    void filterMatches (std::vector<std::vector<cv::DMatch> >& all_matches, std::vector<
            cv::DMatch>& good_matches);

    void getHorizontalMatches (const std::vector<cv::KeyPoint>& template_keypoints,
                               const std::vector<cv::KeyPoint>& search_keypoints,
                               const std::vector<cv::DMatch>& matches,
                               std::vector<cv::DMatch>& horizontal_matches);

    void filterMatchesUsingTracking (const std::vector<cv::KeyPoint>& template_keypoints,
                                     const std::vector<cv::KeyPoint>& search_keypoints,
                                     const std::vector<cv::DMatch>& matches,
                                     std::vector<cv::DMatch>& good_matches,
                                     std::vector<uint>& template_new_to_prev_correspondence,
                                     std::vector<uint>& search_new_to_prev_correspondence);

    void filterMatchesUsingTrackingNN (const std::vector<cv::KeyPoint>& template_keypoints,
                                                const std::vector<cv::KeyPoint>& search_keypoints,
                                                const std::vector<cv::DMatch>& matches,
                                                std::vector<cv::DMatch>& good_matches,
                                                std::vector<uint>& template_new_to_prev_correspondence,
                                                std::vector<uint>& search_new_to_prev_correspondence);

    void filterMatchesUsingTrackingBestMatch (const std::vector<cv::KeyPoint>& template_keypoints,
                                                const std::vector<cv::KeyPoint>& search_keypoints,
                                                const cv::Mat& template_descriptors,
                                                const cv::Mat& search_descriptors,
                                                const std::vector<cv::DMatch>& matches,
                                                std::vector<cv::DMatch>& good_matches,
                                                std::vector<uint>& template_new_to_prev_correspondence,
                                                std::vector<uint>& search_new_to_prev_correspondence);

    void removeOutliers(std::vector<cv::DMatch>& matches, std::vector<double>& distances,
                        std::vector<uint>& template_new_to_prev_correspondence,
                        std::vector<uint>& search_new_to_prev_correspondence);

    void drawInfoOnImg(const int matches, cv::Mat& image);

    void printParameterOptions(cv::Algorithm& cv_algorithm);

    void setParameter(cv::Algorithm& cv_algorithm, std::string param_name, std::string param_value);

    void getDescriptorsInBox(const int x_l, const int x_u, const int y_l, const int y_u,
                             const std::vector<cv::KeyPoint>& target_keypoints,
                             const cv::Mat& target_descriptors,
                             cv::Mat& nearby_descriptors);

    ros::NodeHandle nh_;
    dynamic_reconfigure::Server<feature_cv_waitinglist::FeatureConfig> reconfig_srv_;
    dynamic_reconfigure::Server<feature_cv_waitinglist::FeatureConfig>::CallbackType
        reconfig_callback_;
    ros::Time start_processing_time_, end_processing_time_, publish_time_;

    // class params
    double max_radius_search_dist_, matching_distance_ratio_threshold_,tracking_threshold_, distance_factor_, search_dist_;
    bool distinct_matches_, horizontal_matches_, tracking_matches_,drawing_template_, timing_debug_, outlier_removal_, retain_all_prev_matches_, double_check_tracking_matches_, reduce_search_area_;
    int horizontal_threshold_, min_tracking_matches_;
    int feature_detector_;
    int feature_extractor_;
    int descriptor_matcher_;
    std::string feature_detector_parameter_value_, feature_extractor_parameter_value_;

    std::vector<boost::shared_ptr <cv::FeatureDetector> >     feature_detector_ptrs_;
    std::vector<boost::shared_ptr <cv::DescriptorExtractor> > feature_extractor_ptrs_;
    std::vector<boost::shared_ptr <cv::DescriptorMatcher> >   feature_matcher_ptrs_;

    std::vector<cv::KeyPoint> prev_template_keypoints_, prev_search_keypoints_;
    cv::Mat prev_template_descriptors_, prev_search_descriptors_;
    std::vector<cv::DMatch> prev_matches_;
    bool constructed_;

};

#endif /* FEATUREMATCHING_H_ */
