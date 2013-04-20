/*
 * HomographyEstimation.cpp
 *
 *  Created on: Nov 18, 2012
 *      Author: kidson
 */

#include "feature_cv_waitinglist/homography_estimation.h"

#include <Eigen/SVD>

HomographyEstimation::HomographyEstimation ()
{
  // TODO Auto-generated constructor stub

}

HomographyEstimation::~HomographyEstimation ()
{
  // TODO Auto-generated destructor stub
}


void HomographyEstimation::calculateHomography(const std::vector<FeatureMatch>& input_matches, Eigen::Vector3f& homography_solution)
{
  std::vector<FeatureMatch> normalized_points;
  Eigen::Vector3f normalize_transform;
  normalizePoints(input_matches,normalized_points,normalize_transform);

  Eigen::MatrixXf A(2*input_matches.size(),9);
  std::vector<FeatureMatch>::const_iterator itr;
  for(itr=normalized_points.begin(); itr!=normalized_points.end(); itr++)
  {
    int A_row = (itr - normalized_points.begin())*2;
    Eigen::Vector3f source_point(itr->first->keyPointLocationX, itr->first->keyPointLocationY, 1.0);//(float)itr->first->scale);
    Eigen::Vector3f destination_point(itr->second->keyPointLocationX, itr->second->keyPointLocationY, 1.0);// (float)itr->second->scale);

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
  ROS_INFO_STREAM("V \n" << V);
  //ROS_ERROR_STREAM("S \n" << S);
//  Eigen::JacobiSVD<Eigen::Matrix3f> svd(A);
//  const Eigen::MatrixXf U = svd.matrixU();
//  const Eigen::MatrixXf V = svd.matrixV();
//  const Eigen::VectorXf S = svd.singularValues();

}

void HomographyEstimation::normalizePoints(const std::vector<FeatureMatch>& input_matches, std::vector<FeatureMatch>& output_matches, Eigen::Vector3f & transform){

	double sumX=0, sumY=0;
	double sumXdebug=0, sumYdebug=0;
	double sumDistances=0;

	for(uint i=0; i<input_matches.size();i++){

		FeatureMatch featurematch;
		featurematch.first=new FeatureDescriptorBase(input_matches[i].first);
		featurematch.second=new FeatureDescriptorBase(input_matches[i].second);
		output_matches.push_back(featurematch);

	}


	for(uint i=0; i<output_matches.size();i++){

		FeatureDescriptorBase* feature =output_matches[i].second;
		sumX+=feature->keyPointLocationX;
		sumY+=feature->keyPointLocationY;
//		ROS_ERROR_STREAM("X: "<< feature->keyPointLocationX<< " Y " << feature->keyPointLocationY);

	}

	double avgSumX=sumX/((double)(output_matches.size()));
	double avgSumY=sumY/((double)(output_matches.size()));

//	ROS_ERROR_STREAM("sumes are: "<< sumX<< " and " << sumY);
//	ROS_ERROR_STREAM("size is: " << (double)(output_matches.size()));
//	ROS_ERROR_STREAM("avg sumes are: "<< avgSumX<< " and " << avgSumY);



	for(uint i=0; i<output_matches.size();i++){

		FeatureDescriptorBase* feature =output_matches[i].second;
		feature->keyPointLocationX-=avgSumX;
		feature->keyPointLocationY-=avgSumY;
		feature->scale=1;
		sumXdebug+=feature->keyPointLocationX;
		sumYdebug+=feature->keyPointLocationY;
		sumDistances+=sqrt(feature->keyPointLocationX*feature->keyPointLocationX+feature->keyPointLocationY*feature->keyPointLocationY);
//		ROS_ERROR_STREAM("X: "<< feature->keyPointLocationX<< " Y " << feature->keyPointLocationY);
	}

//	ROS_ERROR_STREAM("debug sumes are: "<< sumXdebug/(double)(output_matches.size()) << " and " << sumYdebug/((double)output_matches.size()));

//	ROS_ERROR_STREAM("distances sumes are: "<< sumDistances/(double)(output_matches.size()));
	double avgSumDistances=sumDistances/(double)(output_matches.size());
	double scale_factor=avgSumDistances/(sqrt(2));

	sumDistances=0.0;
	for(uint i=0; i<output_matches.size();i++){
		FeatureDescriptorBase* feature =output_matches[i].second;
		feature->keyPointLocationX=feature->keyPointLocationX/scale_factor;
		feature->keyPointLocationY=feature->keyPointLocationY/scale_factor;

		sumDistances+=sqrt(feature->keyPointLocationX*feature->keyPointLocationX+feature->keyPointLocationY*feature->keyPointLocationY);
	}

//	ROS_ERROR_STREAM("refined distances sumes are: "<< sumDistances/(double)(output_matches.size()));


	transform[0]=-avgSumX/scale_factor;
	transform[1]=-avgSumY/scale_factor;
	transform[2]=1.0;





}
