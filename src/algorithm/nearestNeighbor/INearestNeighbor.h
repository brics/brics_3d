/**
 * @file 
 * INearestNeighbor.h
 *
 * @date: Jan 5, 2010
 * @author: sblume
 */

#ifndef INEARESTNEIGHBOR_H_
#define INEARESTNEIGHBOR_H_

#include "core/PointCloud3D.h"
#include <vector>

using std::vector;

namespace BRICS_3D {

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
	 * @brief Find the nearest neighbor of the query with respect to the data.
	 *
	 * @param[in] query Content of this vector will be queried to the data.
	 * Make sure it has the same size as the dimension of the data set.
	 * @return Returns the index of the nearest neighbor respect to the data.
	 * If the nearest neighbor exceeds the maximum distance <code> -1 </code> will be returned.
	 *
	 * <b>NOTE:</b> setData() must be invoked before.
	 */
	virtual int findNearestNeighbor(vector<float>* query) = 0;

	/**
	 * @brief Find the nearest neighbor of the query with respect to the data.
	 *
	 * @param[in] query Content of this vector will be queried to the data.
	 * Make sure it has the same size as the dimension of the data set.
	 * @return Returns the index of the nearest neighbor respect to the data.
	 * If the nearest neighbor exceeds the maximum distance <code> -1 </code> will be returned.
	 *
	 * <b>NOTE:</b> setData() must be invoked before.
	 */
	virtual int findNearestNeighbor(vector<double>* query)= 0;

};

}  // namespace BRICS_3D

#endif /* INEARESTNEIGHBOR_H_ */

/* EOF */
