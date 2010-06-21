/**
 * @file 
 * INearestNeighborSetup.h
 *
 * @date: Jun 17, 2010
 * @author: sblume
 */

#ifndef INEARESTNEIGHBORSETUP_H_
#define INEARESTNEIGHBORSETUP_H_


namespace BRICS_3D {

/**
 * @ingroup nearestNeighbor
 * @brief Abstract interface for to configure the Nearest Neighbor search algorithm.
 */
class INearestNeighborSetup {
public:

	/**
	 * @brief Standard constructor.
	 */
	INearestNeighborSetup(){};

	/**
	 * @brief Standard destructor.
	 */
	virtual ~INearestNeighborSetup(){};

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
     * @brief Set the maximum allowed distance to a neighbor.
     *
     * If this value is is set below 0.0 than it is neglected in the search queries.
     * @param maxDistance Set the maximum distance
     */
    void setMaxDistance(double maxDistance)
    {
        this->maxDistance = maxDistance;
    }

protected:

    /// Dimension of data sets search space e.g. 3 for 3D points, etc.
	int dimension;

	/**
	 * @brief Maximal allowed distance to a neighbor. Sometimes also referred as "radius".
	 *
	 * If this value is below 0.0 than it is neglected in the search queries.
	 */
	double maxDistance;
};

}

#endif /* INEARESTNEIGHBORSETUP_H_ */

/* EOF */
