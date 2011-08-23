/*
 * @file:ObjectModelOrientedLine .h
 *
 * @date:Created on: Apr 23, 2011
 * @author:Author: reon
 * @note The implementation is reusing the object model implementation in ROS:PCl
 */

#ifndef OBJECTMODELORIENTEDLINE_H_
#define OBJECTMODELORIENTEDLINE_H_

#include "algorithm/segmentation/objectModels/ObjectModelLine.h"

namespace BRICS_3D {

class ObjectModelOrientedLine : public ObjectModelLine{
private:

	/** @brief The axis along which we need to search for a plane perpendicular to. */
    Eigen::Vector4d axis;

	/** @brief The maximum angle between the model normal and the given axis */
	double epsAngle;

public:
	ObjectModelOrientedLine(){};
	virtual ~ObjectModelOrientedLine(){};

	void
	  selectWithinDistance (const Eigen::VectorXd &model_coefficients, double threshold, std::vector<int> &inliers);

	void
	  getDistancesToModel (const Eigen::VectorXd &model_coefficients, std::vector<double> &distances);
};

}

#endif /* OBJECTMODELORIENTEDLINE_H_ */
