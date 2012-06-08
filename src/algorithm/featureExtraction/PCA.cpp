/******************************************************************************
* BRICS_3D - 3D Perception and Modeling Library
* Copyright (c) 2011, GPS GmbH
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

#include "PCA.h"
#include "core/Logger.h"

namespace BRICS_3D {

PCA::PCA() {

}

PCA::~PCA() {

}

void PCA::computePrincipleComponents(PointCloud3D* inputPointCloud, Eigen::MatrixXd& eigenvectors, Eigen::VectorXd& eigenvalues) {

	// Compute mean and covariance
	Eigen::Matrix3d covariance;
	Eigen::Vector3d centroid;

	centroid = centroidExtractor.computeCentroid(inputPointCloud);

	/*** compute covariance matrix  ***/
	covariance.setZero ();
	for (unsigned int i = 0; i < inputPointCloud->getSize(); ++i) {
		Eigen::Vector4d pt;
		pt[0] = (*inputPointCloud->getPointCloud())[i].getX() - centroid[0];
		pt[1] = (*inputPointCloud->getPointCloud())[i].getY() - centroid[1];
		pt[2] = (*inputPointCloud->getPointCloud())[i].getZ() - centroid[2];
		pt[3] = 1.0; //homogeneous point

		covariance (1, 1) += pt.y () * pt.y (); //the non X parts
		covariance (1, 2) += pt.y () * pt.z ();
		covariance (2, 2) += pt.z () * pt.z ();

		pt *= pt.x ();
		covariance (0, 0) += pt.x (); //the X related parts
		covariance (0, 1) += pt.y ();
		covariance (0, 2) += pt.z ();
	}

	//copy upper triangle to lower triangle as it is symmetric
	covariance (1, 0) = covariance (0, 1);
	covariance (2, 0) = covariance (0, 2);
	covariance (2, 1) = covariance (1, 2);

	/* normalize  */
	covariance /= inputPointCloud->getSize();

	/* compute eigen vectors and values */
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> evd (covariance);
	eigenvalues = evd.eigenvalues().real (); //NOTE eigen values not ordered by absolute values!
	eigenvectors = evd.eigenvectors().real ();

	// sort to descending -> main component is first
	Eigen::VectorXd eigenvaluesTmp = eigenvalues;
	Eigen::MatrixXd eigenvectorsTmp = eigenvectors;
	for (int i = 0; i < 3; ++i) {
		eigenvalues[i] = eigenvaluesTmp[2-i];
		eigenvectors.col (i) = eigenvectorsTmp.col(2-i);
	}

	LOG(DEBUG) << "eigenvalues " << eigenvalues;
	LOG(DEBUG) << "eigenvectors " << eigenvectors;

}

void PCA::computeRotationMatrix(Eigen::MatrixXd eigenvectors, Eigen::VectorXd eigenvalues, IHomogeneousMatrix44* resultRotation) {
	assert(resultRotation != 0);

	double* matrixData;
	matrixData = resultRotation->setRawData();

	/*
	 * column-row layout:
	 * 0 4 8  12
	 * 1 5 9  13
	 * 2 6 10 14
	 * 3 7 11 15
	 */
	/* rotation */
	matrixData[0] = eigenvectors(0,0);
	matrixData[4] = eigenvectors(0,1);
	matrixData[8] = eigenvectors(0,2);
	matrixData[1] = eigenvectors(1,0);
	matrixData[5] = eigenvectors(1,1);
	matrixData[9] = eigenvectors(1,2);
	matrixData[2] = eigenvectors(2,0);
	matrixData[6] = eigenvectors(2,1);
	matrixData[10] = eigenvectors(2,2);

	/* translation */
	matrixData[12] = 0.0;
	matrixData[13] = 0.0;
	matrixData[14] = 0.0;

	/* homogeneous coefficients */
	matrixData[3] = 0.0;
	matrixData[7] = 0.0;
	matrixData[11] = 0.0;
	matrixData[15] = 1.0;

}

}

/* EOF */
