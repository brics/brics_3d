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

#ifndef BRICS_3D_INEARESTPOINT3DNEIGHBOR_H_
#define BRICS_3D_INEARESTPOINT3DNEIGHBOR_H_

#include "brics_3d/core/PointCloud3D.h"
#include <vector>

using std::vector;

namespace brics_3d {

/**
 * @ingroup nearestNeighbor
 * @brief Abstract interface for the nearest neighbor search algorithm for 3D points.
 *
 *  The interface INearestPoint3DNeighbor is specific to the 3D perception and modeling
 *  domain, as it uses Cartesian points.
 *
 */
class INearestPoint3DNeighbor {
public:

	/**
	 * @brief Standard constructor.
	 */
	INearestPoint3DNeighbor(){};

	/**
	 * @brief Standard destructor.
	 */
	virtual ~INearestPoint3DNeighbor(){};

	/**
	 * @brief Set the data
	 *
	 * @param[in] data Data is represented as a PointCloud3D.
	 */
	virtual void setData(PointCloud3D* data)= 0;

	/**
	 * @brief Find the nearest neighbor point of the query with respect to the data point cloud.
	 *
	 * @param[in] query Point that will be queried to the data.
	 * @param[out] resultIndices Returns the indices of the $k$ nearest neighbors with respect to the data point cloud.
	 * If the nearest neighbor exceeds the maximum distance the returned vector will be empty.
	 * @param[in] k Sets how many nearest neighbors will be searched.
	 *
	 * <b>NOTE:</b> setData() must be invoked before.
	 */
	virtual void findNearestNeighbors(Point3D* query, std::vector<int>* resultIndices, unsigned int k = 1) = 0; //TODO typo: findNearestNeighbors
};

}  // namespace brics_3d

#endif /* BRICS_3D_INEARESTPOINT3DNEIGHBOR_H_ */

/* EOF */
