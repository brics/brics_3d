/******************************************************************************
* BRICS_3D - 3D Perception and Modeling Library
* Copyright (c) 2014, KU Leuven
*
* Author: Sebastian Blumenthal
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

#ifndef COVARIANCE3D_H_
#define COVARIANCE3D_H_

#include "brics_3d/core/PointCloud3D.h"

#include <Eigen/Geometry>
#include <Eigen/Dense>

namespace brics_3d {

/**
 * @brief Covariance calculation for a given point cloud
 * @ingroup featureExtraction
 *
 * Based on PCL Feature code.
 */
class Covariance3D {
public:
	Covariance3D();
	virtual ~Covariance3D();

	/**
	 * @brief Calculate the 3x3 covariance matrix for a given point cloud.
	 * @param cloud Input point cloud.
	 * @return The covariance matrix as 3x3 eigen matrix
	 */
	static Eigen::Matrix3d computeCovarianceMatrix (PointCloud3D* cloud, const Eigen::Vector4d &centroid);

	/**
	 * @brief Calculate the 3x3 covariance matrix for a given point cloud with indices.
	 * @param cloud Input point cloud.
	 * @param indices Intidaces the subset of the point cloud to be used.
	 * @return The covariance matrix as 3x3 eigen matrix
	 */
	static Eigen::Matrix3d computeCovarianceMatrix (PointCloud3D* cloud, const std::vector<int> &indices, const Eigen::Vector4d &centroid);
};

} /* namespace brics_3d */

#endif /* COVARIANCE3D_H_ */
