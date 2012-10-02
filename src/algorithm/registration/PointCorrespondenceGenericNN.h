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

#ifndef BRICS_3D_POINTCORRESPONDENCEGENERICNN_H_
#define BRICS_3D_POINTCORRESPONDENCEGENERICNN_H_

#include "IPointCorrespondence.h"
#include "algorithm/nearestNeighbor/INearestPoint3DNeighbor.h"

namespace brics_3d {

/**
 * @ingroup registration
 * @brief Implementation of correspondence problem for points using generic nearest neighbor search
 */
class PointCorrespondenceGenericNN: public brics_3d::IPointCorrespondence {
public:

	/**
	 * @brief Standard constructor
	 */
	PointCorrespondenceGenericNN();

	/**
	 * @brief Constructor that defines the nearest neighbor search strategy
	 * @param[in] nearestNeighborAlgorithm The nearest neighbor search strategy that will be used
	 */
	PointCorrespondenceGenericNN(INearestPoint3DNeighbor* nearestNeighborAlgorithm);

	/**
	 * @brief Standard destructor
	 */
	virtual ~PointCorrespondenceGenericNN();

	void createNearestNeighborCorrespondence(PointCloud3D* pointCloud1, PointCloud3D* pointCloud2,
			std::vector<CorrespondencePoint3DPair>* resultPointPairs);
	/**
	 * @brief Get the nearest neighbor search strategy
	 * @return Returns the nearest neighbor search strategy
	 */
	INearestPoint3DNeighbor* getNearestNeighborAlgorithm() const;

	/**
	 * @brief Set the nearest neighbor search strategy
	 * @param[in] nearestNeighborAlgorithm The nearest neighbor search strategy that will be used
	 */
	void setNearestNeighborAlgorithm(INearestPoint3DNeighbor* nearestNeighborAlgorithm);

private:

	/// Internal handle to the nearest neighbor search strategy
	INearestPoint3DNeighbor* nearestNeighborAlgorithm;
};

}

#endif /* BRICS_3D_POINTCORRESPONDENCEGENERICNN_H_ */

/* EOF */
