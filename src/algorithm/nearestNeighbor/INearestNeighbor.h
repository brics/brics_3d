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
 * @brief Abstract interface for the nearest neighbor search algorithm
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
	 * @brief Set the data
	 *
	 * @param[in] data Data is represented as a PointCloud3D.
	 */
	virtual void setData(PointCloud3D* data)= 0;

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
	virtual int findNearestNeigbor(vector<float>* query) = 0;

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
	virtual int findNearestNeigbor(vector<double>* query)= 0;

	/**
	 * @brief Find the nearest neighbor point of the query with respect to the data point cloud.
	 *
	 * @param[in] query Point that will be queried to the data.
	 * @return Returns the index of the nearest neighbor respect to the data point cloud.
	 * If the nearest neighbor exceeds the maximum distance <code> -1 </code> will be returned.
	 *
	 * <b>NOTE:</b> setData() must be invoked before.
	 */
	virtual int findNearestNeigbor(Point3D* query)= 0;

	/**
	 * @brief Get the current dimension of the data set
	 *
	 * Size of the dimension is automatically determined by setData() or is <code> -1 </code> if no data is present.
	 * @return Returns the dimension
	 */
    int getDimension() const
    {
        return dimension;
    }

    /**
     * @brief Get the maximum allowed distance to a neighbor
     * @return Returns the maximum distance
     */
    double getMaxDistance() const
    {
        return maxDistance;
    }

    /**
     * @brief Set the maximum allowed distance to a neighbor
     * @param maxDistance Set the maximum distance
     */
    void setMaxDistance(double maxDistance)
    {
        this->maxDistance = maxDistance;
    }

protected:

    /// Dimension of data sets search space e.g. 3 for 3D points, etc.
	int dimension;

	/// Maximal allowed distance to a neighbor
	double maxDistance;
};

}  // namespace BRICS_3D

#endif /* INEARESTNEIGHBOR_H_ */

/* EOF */
