/*
 * HomographyEstimation.h
 *
 *  Created on: Nov 18, 2012
 *      Author: kidson
 */

#ifndef HOMOGRAPHYESTIMATION_H_
#define HOMOGRAPHYESTIMATION_H_

#include "FeatureMatch.h"
#include <ros/console.h>
#include <Eigen/Core>

class HomographyEstimation
{
  public:
    HomographyEstimation ();
    virtual ~HomographyEstimation ();

    void calculateHomography(const std::vector<FeatureMatch>& input_matches, Eigen::Vector3f& homography_solution);
private:
    void normalizePoints(const std::vector<FeatureMatch>& input_matches, std::vector<FeatureMatch>& output_matches, Eigen::Vector3f & transform);
};

#endif /* HOMOGRAPHYESTIMATION_H_ */
