/*
 * ransac_transformation.cpp
 *
 *  Created on: Dec 17, 2012
 *      Author: Karol Hausman
                Ross Kidson
 */

#include <ransac_waitinglist/ransac_transformation.h>

#include <pcl/registration/transformation_estimation_svd.h>

RANSACTransformation::RANSACTransformation():
    nh_ ("~/ransac_tranformation"),
    reconfig_srv_(nh_)
{
    reconfig_callback_ = boost::bind (&RANSACTransformation::reconfigCallback, this, _1, _2);
    reconfig_srv_.setCallback (reconfig_callback_);
}

RANSACTransformation::~RANSACTransformation()
{

}

void RANSACTransformation::reconfigCallback (ransac_waitinglist::RANSACConfig&config, uint32_t level){

    ransac_max_distance_inliers_=config.ransac_max_distance_inliers;
    ransac_iterations_=config.ransac_iterations;
    neighbor_distance_threshold_=config.neighbor_distance_threshold;
    pcl_matches_min_number_ = config.pcl_matches_min_number;
    refine_transform_ = config.refine_transform;
}

void RANSACTransformation::calcPointsDistance(const pcl::PointXYZ &point_source,const pcl::PointXYZ &point_destination,double &distance){

    distance=sqrt((point_source.x-point_destination.x)*(point_source.x-point_destination.x)+(point_source.y-point_destination.y)*(point_source.y-point_destination.y)
                  +(point_source.z-point_destination.z)*(point_source.z-point_destination.z));
}

int RANSACTransformation::ransacUmeyama(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud_source,
                                        const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud_destination,
                                        Eigen::Matrix4f& transformation_final_result){

    std::vector<int> indices_source, inliers;

    Eigen::Matrix4f transformationResult;
    int max_inliers_number=0;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source_temp(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_destination_temp(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source_transformed(new pcl::PointCloud<pcl::PointXYZ>);

    int winner_idx=0;
    //Ransac loop
    for(int i=0;i<ransac_iterations_;i++){
        indices_source.clear();
        cloud_source_temp->clear();
        cloud_destination_temp->clear();
        cloud_source_transformed->clear();

        //get random subset of the pointcloud
        for (int j=0;j<pcl_matches_min_number_;j++){
            int rand_int= rand()%static_cast<int>(cloud_source->points.size());
            indices_source.push_back(rand_int);
        }

        pcl::copyPointCloud(*cloud_source,indices_source,*cloud_source_temp);
        pcl::copyPointCloud(*cloud_destination,indices_source,*cloud_destination_temp);

        umeyamaTransform(cloud_source_temp,cloud_destination_temp,transformationResult);

        pcl::transformPointCloud(*cloud_source,*cloud_source_transformed,transformationResult);

        getInliers(cloud_source_transformed, cloud_destination, inliers);

        const double percentage_inliers = ((double)max_inliers_number / (double)cloud_source->size());
        if((int)inliers.size() > max_inliers_number)
        {
            max_inliers_number=inliers.size();
            transformation_final_result = transformationResult;
            winner_idx = i;
        }

//         //speed hacks
//        if(percentage_inliers > 0.5)
//            i++;      //2x as fast
//        else if(percentage_inliers > 0.75)
//            i+=3;     //4x as fast
    }
    ROS_DEBUG_STREAM("inliers: " << max_inliers_number << ((double)max_inliers_number / (double)cloud_source->size()) * 100 << "%");
    ROS_DEBUG_STREAM("winner at iteration " << winner_idx);

    pcl::transformPointCloud(*cloud_source,*cloud_source_transformed,transformation_final_result);

    getInliers(cloud_source_transformed, cloud_destination, inliers);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source_final(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_destination_final(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::copyPointCloud(*cloud_source,inliers,*cloud_source_final);
    pcl::copyPointCloud(*cloud_destination,inliers,*cloud_destination_final);

    if(refine_transform_)
        umeyamaTransform(cloud_source_final,cloud_destination_final,transformation_final_result);
    return max_inliers_number;
}

void RANSACTransformation::getInliers(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud_source_transformed,
                                      const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud_destination,
                                      std::vector<int>& inlier_indices)
{
    inlier_indices.clear();
    double distance(0.0);
    for (int pcl_it = 0; pcl_it< static_cast<int> (cloud_source_transformed->points.size ()); ++pcl_it){

        const pcl::PointXYZ& point_source = cloud_source_transformed->points[pcl_it];
        const pcl::PointXYZ& point_dest=cloud_destination->points[pcl_it];
        calcPointsDistance(point_source,point_dest,distance);
        if(distance<ransac_max_distance_inliers_){
            inlier_indices.push_back(pcl_it);
        }
    }
}


void RANSACTransformation::umeyamaTransform (const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_source,
                                       const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_destination,
                                       Eigen::Matrix4f& transformationResult)
{
    pcl::registration::TransformationEstimationSVD<pcl::PointXYZ,pcl::PointXYZ> svd;
    svd.estimateRigidTransformation(*cloud_source, *cloud_destination, transformationResult);
}

bool RANSACTransformation::extractNeighbor(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_source,
                                           pcl::PointXYZ &searchPoint,
                                           pcl::PointXYZ &found_point)
{
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
    kdtree->setInputCloud(cloud_source);

    std::vector<int> pointIdxNKNSearch;
    std::vector<float> pointNKNSquaredDistance;
    pointIdxNKNSearch.resize(1);
    pointNKNSquaredDistance.resize(1);

    if (kdtree->nearestKSearch(searchPoint, 1, pointIdxNKNSearch,
                               pointNKNSquaredDistance) > 0) {
        for (size_t i = 0; i < pointIdxNKNSearch.size(); ++i){

            if(pointNKNSquaredDistance[i]<neighbor_distance_threshold_){
                found_point=cloud_source->points[pointIdxNKNSearch[i]];
                return true;
            }
        }
    }
    return false;
}
