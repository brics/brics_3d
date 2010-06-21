/**
 * @file 
 * INearestPoint3DNeighbor.h
 *
 * @date: Jun 17, 2010
 * @author: sblume
 */

#ifndef INEARESTPOINT3DNEIGHBOR_H_
#define INEARESTPOINT3DNEIGHBOR_H_

#include "core/PointCloud3D.h"
#include <vector>

using std::vector;

namespace BRICS_3D {

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
	virtual void findNearestNeigbor(Point3D* query, std::vector<int>* resultIndices, unsigned int k = 1) = 0; //TODO typo: findNearestNeighbor
};

}  // namespace BRICS_3D

#endif /* INEARESTPOINT3DNEIGHBOR_H_ */

/* EOF */
