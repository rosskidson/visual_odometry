/*
 * ransac_transformation.h
 *
 *  Created on: Dec 17, 2012
 *      Author: Karol Hausman
                Ross Kidson
 */


#include <dynamic_reconfigure/server.h>
#include "../../cfg/cpp/ransac_waitinglist/RANSACConfig.h"

#include <pcl/point_types.h>

#ifndef RANSAC_TRANSFORMATION_H_
#define RANSAC_TRANSFORMATION_H_

class RANSACTransformation
{

public:

    RANSACTransformation();
    virtual ~RANSACTransformation();

    int ransacUmeyama(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud_source,
                      const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud_destination,
                      Eigen::Matrix4f& transformation_final_result);

private:
  void reconfigCallback (ransac_waitinglist::RANSACConfig&config, uint32_t level);

  void calcPointsDistance(const pcl::PointXYZ &point_source,const pcl::PointXYZ &point_destination,double &distance);


  bool extractNeighbor(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_source, pcl::PointXYZ &searchPoint,
                  pcl::PointXYZ &found_point);

  void umeyamaTransform (const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_source,
                                          const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_destination,
                                          Eigen::Matrix4f& transformationResult);

  void getInliers(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud_source_transformed,
                                        const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud_destination,
                                        std::vector<int>& inlier_indices);

  ros::NodeHandle nh_;

  dynamic_reconfigure::Server<ransac_waitinglist::RANSACConfig> reconfig_srv_;
  dynamic_reconfigure::Server<ransac_waitinglist::RANSACConfig>::CallbackType
  reconfig_callback_;
  int pcl_matches_min_number_;
  double ransac_max_distance_inliers_;
  int ransac_iterations_;
  double neighbor_distance_threshold_;
  bool refine_transform_;

};

#endif /* RANSAC_TRANSFORMATION_H_ */

