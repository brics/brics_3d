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

#include "Covariance3D.h"

namespace brics_3d {

Covariance3D::Covariance3D() {

}

Covariance3D::~Covariance3D() {

}

Eigen::Matrix3d Covariance3D::computeCovarianceMatrix(PointCloud3D* cloud, const Eigen::Vector4d& centroid) {
	Eigen::Matrix3d covariance_matrix;
	covariance_matrix.setZero ();

	if (cloud->getSize() == 0) {
		return Eigen::Matrix3d(); //"null" object
	}

	for (size_t i = 0; i < cloud->getSize(); ++i) {
		// Check if the point is invalid
		if ( isnan( (*cloud->getPointCloud())[i].getX() ) || isnan( (*cloud->getPointCloud())[i].getY() ) || isnan( (*cloud->getPointCloud())[i].getZ() ))
			continue;

		Point3D p;
		p.setX( (*cloud->getPointCloud())[i].getX() - centroid[0]);
		p.setY( (*cloud->getPointCloud())[i].getY() - centroid[1]);
		p.setZ( (*cloud->getPointCloud())[i].getZ() - centroid[2]);

		double demean_xy = p.getX() * p.getY();
		double demean_xz = p.getX() * p.getZ();
		double demean_yz = p.getY() * p.getZ();

		covariance_matrix (0, 0) += p.getX() * p.getX();
		covariance_matrix (0, 1) += demean_xy;
		covariance_matrix (0, 2) += demean_xz;
		covariance_matrix (1, 0) += demean_xy;
		covariance_matrix (1, 1) += p.getY() * p.getY();
		covariance_matrix (1, 2) += demean_yz;
		covariance_matrix (2, 0) += demean_xz;
		covariance_matrix (2, 1) += demean_yz;
		covariance_matrix (2, 2) += p.getZ() * p.getZ();
	}

	return covariance_matrix;
}

Eigen::Matrix3d Covariance3D::computeCovarianceMatrix(PointCloud3D* cloud, const std::vector<int>& indices, const Eigen::Vector4d& centroid) {
	Eigen::Matrix3d covariance_matrix;
	covariance_matrix.setZero ();

	if (indices.size () == 0) {
		return Eigen::Matrix3d(); //"null" object
	}

	// For each point in the cloud
	for (size_t i = 0; i < indices.size (); ++i) {
		// Check if the point is invalid
		if ( isnan( (*cloud->getPointCloud())[indices[i]].getX() ) || isnan( (*cloud->getPointCloud())[indices[i]].getY() ) || isnan( (*cloud->getPointCloud())[indices[i]].getZ() ))
			continue;

		Point3D p;
		p.setX( (*cloud->getPointCloud())[indices[i]].getX() - centroid[0]);
		p.setY( (*cloud->getPointCloud())[indices[i]].getY() - centroid[1]);
		p.setZ( (*cloud->getPointCloud())[indices[i]].getZ() - centroid[2]);

		double demean_xy = p.getX() * p.getY();
		double demean_xz = p.getX() * p.getZ();
		double demean_yz = p.getY() * p.getZ();

		covariance_matrix (0, 0) += p.getX() * p.getX();
		covariance_matrix (0, 1) += demean_xy;
		covariance_matrix (0, 2) += demean_xz;
		covariance_matrix (1, 0) += demean_xy;
		covariance_matrix (1, 1) += p.getY() * p.getY();
		covariance_matrix (1, 2) += demean_yz;
		covariance_matrix (2, 0) += demean_xz;
		covariance_matrix (2, 1) += demean_yz;
		covariance_matrix (2, 2) += p.getZ() * p.getZ();
	}

	return covariance_matrix;
}

} /* namespace brics_3d */
