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

#ifndef BRICS_3D_PCA_H_
#define BRICS_3D_PCA_H_

#include <Eigen/StdVector>
#include "brics_3d/core/PointCloud3D.h"
#include "brics_3d/core/IHomogeneousMatrix44.h"
#include "brics_3d/core/IPoint3DIterator.h"
#include "brics_3d/algorithm/featureExtraction/Centroid3D.h"

namespace brics_3d {

/**
 * @brief Principle Component Analysis for point clouds.
 * @ingroup featureExtraction
 */
class PCA {
public:

	/**
	 * Standard constructor.
	 */
	PCA();

	/**
	 * Standard destructor
	 */
	virtual ~PCA();

	/**
	 * Compute the principle components for a point cloud.
	 * @param[in] inputPointCloud The input point cloud.
	 * @param[in] eigenvectors Eigen vectors representing the principle components. Sorted in descending order of eigen values.
	 * @param[out] eigenvalues Corresponding eingen values. Sorted in descending order.
	 */
	void computePrincipleComponents(PointCloud3D* inputPointCloud, Eigen::MatrixXd& eigenvectors, Eigen::VectorXd& eigenvalues);

	/// Same as above but with iterator as interface.
	void computePrincipleComponents(IPoint3DIterator::IPoint3DIteratorPtr inputPointCloud, Eigen::MatrixXd& eigenvectors, Eigen::VectorXd& eigenvalues);


	/**
	 * Given the principle components compute a transform.
	 * @param[in] eigenvectors Eigen vectors representing the principle components.
	 * @param[in] eigenvalues Corresponding eingen values.
	 * @param[out] resultRotation The resulting transform.
	 */
	void computeRotationMatrix(Eigen::MatrixXd eigenvectors, Eigen::VectorXd eigenvalues, IHomogeneousMatrix44* resultRotation);

private:

	///Centroid extraction that is needed for PCA.
	Centroid3D centroidExtractor;
};

}

#endif /* BRICS_3D_PCA_H_ */

/* EOF */
