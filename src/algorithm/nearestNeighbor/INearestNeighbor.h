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

#ifndef BRICS_3D_INEARESTNEIGHBOR_H_
#define BRICS_3D_INEARESTNEIGHBOR_H_

#include "core/PointCloud3D.h"
#include <vector>

using std::vector;

namespace brics_3d {

/**
 * @ingroup nearestNeighbor
 * @brief Abstract interface for the Nearest Neighbor search algorithm
 *
 */
class INearestNeighbor {
public:

	/**
	 * @brief Standard constructor
	 */
	INearestNeighbor(){};

	/**
	 * @brief Standard destructor
	 */
	virtual ~INearestNeighbor(){};

	/**
	 * @brief Set the data
	 *
	 * @param[in] data Data matrix. The inner vector forms the dimension of the search space and must be constant.
	 */
	virtual void setData(vector< vector<float> >* data) = 0;

	/**
	 * @brief Set the data
	 *
	 * @param[in] data Data matrix. The inner vector forms the dimension of the search space and must be constant.
	 */
	virtual void setData(vector< vector<double> >* data) = 0;

	/**
	 * @param[in] query Vector that will be queried to the data.
	 * Please make sure that it has the dame dimensionality as the data, set with setData(). Otherwise an
	 * exception is thrown.
	 * @param[out] resultIndices Returns the indices of the $k$ nearest neighbors with respect to the data point cloud.
	 * If the nearest neighbor exceeds the maximum distance the returned vector will be empty.
	 * @param[in] k Sets how many nearest neighbors will be searched.
	 *
	 * <b>NOTE:</b> setData() must be invoked before.
	 */
	virtual void findNearestNeighbors(vector<float>* query, std::vector<int>* resultIndices, unsigned int k = 1) = 0;

	/**
	 * @param[in] query Vector that will be queried to the data.
	 * Please make sure that it has the dame dimensionality as the data, set with setData(). Otherwise an
	 * exception is thrown.
	 * @param[out] resultIndices Returns the indices of the $k$ nearest neighbors with respect to the data point cloud.
	 * If the nearest neighbor exceeds the maximum distance the returned vector will be empty.
	 * @param[in] k Sets how many nearest neighbors will be searched.
	 *
	 * <b>NOTE:</b> setData() must be invoked before.
	 */
	virtual void findNearestNeighbors(vector<double>* query, std::vector<int>* resultIndices, unsigned int k = 1) = 0;

};

}  // namespace brics_3d

#endif /* BRICS_3D_INEARESTNEIGHBOR_H_ */

/* EOF */
