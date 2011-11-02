/******************************************************************************
* BRICS_3D - 3D Perception and Modeling Library
* Copyright (c) 2011, GPS GmbH
*
* Author: Pinaki Sunil Banerjee
*
*
* This software is published under a dual-license: GNU Lesser General Public
* License LGPL 2.1 and Modified BSD license. The dual-license implies that
* users of this code may choose which terms they prefer.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU Lesser General Public License LGPL and the BSD license for
* more details.
*
******************************************************************************/

#ifndef OBJECTMODELORIENTEDLINE_H_
#define OBJECTMODELORIENTEDLINE_H_

#include "algorithm/segmentation/objectModels/ObjectModelLine.h"

namespace BRICS_3D {

/**
 * @note The implementation is reusing the object model implementation in ROS:PCl
 */
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
