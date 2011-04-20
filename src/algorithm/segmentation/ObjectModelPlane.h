/*
 * ObjectModelPlane.h
 *
 *  Created on: Apr 17, 2011
 *      Author: reon
 */

#ifndef OBJECTMODELPLANE_H_
#define OBJECTMODELPLANE_H_
#include "algorithm/segmentation/IObjectModel.h"
#include "core/Point3D.h"
#include "core/PointCloud3D.h"

/**
 * Implementation of IObjectModel for SAC-Model Plane from ROS-PCL.
 */


namespace BRICS_3D {


/** \brief Get the distance from a point to a plane (signed) defined by ax+by+cz+d=0
 * \param p a point
 * \param a the normalized <i>a</i> coefficient of a plane
 * \param b the normalized <i>b</i> coefficient of a plane
 * \param c the normalized <i>c</i> coefficient of a plane
 * \param d the normalized <i>d</i> coefficient of a plane
 */
inline double
pointToPlaneDistanceSigned (const Point3D &p, double a, double b, double c, double d)
{
	return (a * p.getX() + b * p.getY() + c * p.getZ() + d);
}

/** \brief Get the distance from a point to a plane (signed) defined by ax+by+cz+d=0
 * \param p a point
 * \param plane_coefficients the normalized coefficients (a, b, c, d) of a plane
 */
inline double
pointToPlaneDistanceSigned (const Point3D &p, const Eigen::Vector4f &plane_coefficients)
{
	return ( plane_coefficients[0] * p.getX() + plane_coefficients[1] * p.getY() + plane_coefficients[2] * p.getZ() + plane_coefficients[3] );
}






class ObjectModelPlane:public IObjectModel {

private:
	/** \brief Define the maximum number of iterations for collinearity checks */
	const static int MAX_ITERATIONS_COLLINEAR = 1000;
	bool isInitialised;
	std::vector<Point3D> *points;	/** Pointer to the vector of points in the pointcloud**/

public:
	ObjectModelPlane():IObjectModel(){};
//	ObjectModelPlane(PointCloud3D* p);
//	ObjectModelPlane(PointCloud3D* p, std::vector<int> &indices);

	void getSamples (int &iterations, std::vector<int> &samples);
	bool computeModelCoefficients (const std::vector<int> &samples, Eigen::VectorXf &model_coefficients);
	void optimizeModelCoefficients (const std::vector<int> &inliers, const Eigen::VectorXf &model_coefficients,
			Eigen::VectorXf &optimized_coefficients);
	void getDistancesToModel (const Eigen::VectorXf &model_coefficients, std::vector<double> &distances);
	void selectWithinDistance (const Eigen::VectorXf &model_coefficients, double threshold,
			std::vector<int> &inliers);
	void getInlierDistance (std::vector<int> &inliers, const Eigen::VectorXf &model_coefficients,  std::vector<double> &distances);
	void projectPoints (const std::vector<int> &inliers, const Eigen::VectorXf &model_coefficients,
			PointCloud3D* projectedPointCloud);
	bool doSamplesVerifyModel (const std::set<int> &indices, const Eigen::VectorXf &model_coefficients, double threshold);

	virtual ~ObjectModelPlane(){};
};

}

#endif /* OBJECTMODELPLANE_H_ */
