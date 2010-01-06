/**
 * @file 
 * NearestNeighborFLANN.h
 *
 * @date: Jan 5, 2010
 * @author: sblume
 */

#ifndef NEARESTNEIGHBORFLANN_H_
#define NEARESTNEIGHBORFLANN_H_

#include "algorithm/nearestNeighbor/INearestNeighbor.h"
#include "flann/src/cpp/flann.h"

namespace BRICS_3D {

/**
 * @ingroup nearestNeighbor
 * @brief Implementaion for the nearest neighbor search algorithm with the FĹANN library.
 *
 */
class NearestNeighborFLANN : public INearestNeighbor {
public:

	/**
	 * @brief Standard constructor
	 */
	NearestNeighborFLANN();

	/**
	 * @brief Standard destructor
	 */
	virtual ~NearestNeighborFLANN();

	void setData(vector< vector<float> >* data);
	void setData(vector< vector<double> >* data);
	void setData(PointCloud3D* data);

	int findNearestNeigbor(vector<float>* query);
	int findNearestNeigbor(vector<double>* query);
	int findNearestNeigbor(Point3D* query);

private:

	/// Matrix in major-row representation
	float* dataMatrix;

	/// Number of rows in the dataMatrix
	int rows;

	/// Number of columns in the dataMatrix
	int cols;

	/// Container for various algorithm parameters
	FLANNParameters p;

	/// Handle for the reprocessed data (e.g. k-d tree)
	FLANN_INDEX index_id;

	/// Estimated speedup of used algorithm with respect to a brute force approach
	float speedup;
};

}

#endif /* NEARESTNEIGHBORFLANN_H_ */

/* EOF */
