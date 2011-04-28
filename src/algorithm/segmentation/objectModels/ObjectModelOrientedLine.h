/*
 * ObjectModelOrientedLine.h
 *
 *  Created on: Apr 23, 2011
 *      Author: reon
 */

#ifndef OBJECTMODELORIENTEDLINE_H_
#define OBJECTMODELORIENTEDLINE_H_

#include "algorithm/segmentation/objectModels/ObjectModelLine.h"

namespace BRICS_3D {

class ObjectModelOrientedLine : public ObjectModelLine{
private:

	/** \brief The axis along which we need to search for a plane perpendicular to. */
     Eigen::Vector4d axis;

     /** \brief The maximum allowed difference between the plane normal and the given axis. */
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
