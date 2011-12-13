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

#ifndef OBJECTMODELNORMALPLANE_H_
#define OBJECTMODELNORMALPLANE_H_


#include "algorithm/segmentation/objectModels/IObjectModelUsingNormals.h"


namespace BRICS_3D {

/**
 * @note The implementation is reusing the object model implementation in ROS:PCl
 */
class ObjectModelNormalPlane : public IObjectModelUsingNormals{

private:

	/** @brief The axis along which we need to search for a plane perpendicular to. */
    Eigen::Vector4d axis;

    /** @brief The distance from the template plane to the origin. */
    double distanceFromOrigin;

    /** @brief The maximum allowed difference between the plane normal and the given axis. */
    double epsAngle;

    /** @brief The maximum allowed deviation from the template distance from the origin. */
    double epsDistance;


public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ObjectModelNormalPlane(){};
	virtual ~ObjectModelNormalPlane(){};

    /** @brief Set the axis along which we need to search for a plane perpendicular to.
      * @param ax the axis along which we need to search for a plane perpendicular to
      */
    inline void
      setAxis (const Eigen::Vector3d &ax)
    {
#ifdef EIGEN3
		axis.head<3> () = ax;
#else
		axis.start<3> () = ax;
#endif
      axis[3] = 0;
    }


	/** @brief Set the angle epsilon (delta) threshold.
      * @param ea the maximum allowed difference between the plane normal and the given axis.
      */
    inline void
      setEpsAngle (double ea)
    {
      this->epsAngle = ea;
    }

    /** @brief Set the distance we expect the plane to be from the origin
      * @param d distance from the template plane to the origin
      */
    inline void
      setDistanceFromOrigin (double d)
    {
      this->distanceFromOrigin = d;
    }


    /** @brief Get the distance of the plane from the origin. */
    inline double
      getDistanceFromOrigin ()
    {
      return (this->distanceFromOrigin);
    }


    /** @brief Set the distance epsilon (delta) threshold.
      * @param delta the maximum allowed deviation from the template distance from the origin
      */
    inline void
      setEpsDist (double delta)
    {
      this->epsDistance = delta;
    }

    /** @brief Get the distance epsilon (delta) threshold. */
    inline double
      getEpsDist ()
    {
      return (this->epsDistance);
    }


	void getSamples (int &iterations, std::vector<int> &samples);
	bool computeModelCoefficients (const std::vector<int> &samples, Eigen::VectorXd &model_coefficients);
	void optimizeModelCoefficients (const std::vector<int> &inliers, const Eigen::VectorXd &model_coefficients,
			Eigen::VectorXd &optimized_coefficients);
	void getDistancesToModel (const Eigen::VectorXd &model_coefficients, std::vector<double> &distances);
	void selectWithinDistance (const Eigen::VectorXd &model_coefficients, double threshold,
			std::vector<int> &inliers);
	void getInlierDistance (std::vector<int> &inliers, const Eigen::VectorXd &model_coefficients,  std::vector<double> &distances);
	void projectPoints (const std::vector<int> &inliers, const Eigen::VectorXd &model_coefficients,
			PointCloud3D* projectedPointCloud);
	bool doSamplesVerifyModel (const std::set<int> &indices, const Eigen::VectorXd &model_coefficients, double threshold);

	void computeRandomModel (int &iterations, Eigen::VectorXd &model_coefficients, bool &isDegenerate,
			bool &modelFound);

	inline int getNumberOfSamplesRequired(){return 3;};
};

}

#endif /* OBJECTMODELNORMALPLANE_H_ */
