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

#ifndef OBJECTMODELPLANE_H_
#define OBJECTMODELPLANE_H_
#include "algorithm/segmentation/objectModels/IObjectModel.h"


namespace brics_3d {


/** @brief Get the distance from a point to a plane (signed) defined by ax+by+cz+d=0
 *  @param p a point
 *  @param a the normalized <i>a</i> coefficient of a plane
 *  @param b the normalized <i>b</i> coefficient of a plane
 *  @param c the normalized <i>c</i> coefficient of a plane
 *  @param d the normalized <i>d</i> coefficient of a plane
 */
inline double
pointToPlaneDistanceSigned (const Point3D &p, double a, double b, double c, double d)
{
	return (a * p.getX() + b * p.getY() + c * p.getZ() + d);
}

/** @brief Get the distance from a point to a plane (signed) defined by ax+by+cz+d=0
 *  @param p a point from which the distance is to be measured
 *  @param plane_coefficients the normalized coefficients (a, b, c, d) of a plane
 */
inline double
pointToPlaneDistanceSigned (const Point3D &p, const Eigen::Vector4d &plane_coefficients)
{
	return ( plane_coefficients[0] * p.getX() + plane_coefficients[1] * p.getY() + plane_coefficients[2] * p.getZ() + plane_coefficients[3] );
}



/**
 * @note The implementation is reusing the object model implementation in ROS:PCl
 * @ingroup segmentation
 */
class ObjectModelPlane:public IObjectModel {

public:
	ObjectModelPlane():IObjectModel(){
	};

	void getSamples (int &iterations, std::vector<int> &samples);
	bool computeModelCoefficients (const std::vector<int> &samples, Eigen::VectorXd &model_coefficients);
	void getDistancesToModel (const Eigen::VectorXd &model_coefficients, std::vector<double> &distances);
	void selectWithinDistance (const Eigen::VectorXd &model_coefficients, double threshold,
			std::vector<int> &inliers);
	void getInlierDistance (std::vector<int> &inliers, const Eigen::VectorXd &model_coefficients,  std::vector<double> &distances);
	bool doSamplesVerifyModel (const std::set<int> &indices, const Eigen::VectorXd &model_coefficients, double threshold);

	void computeRandomModel (int &iterations, Eigen::VectorXd &model_coefficients, bool &isDegenerate, bool &modelFound);

	inline int getNumberOfSamplesRequired(){return 3;};

	virtual ~ObjectModelPlane(){};
};

}

#endif /* OBJECTMODELPLANE_H_ */
