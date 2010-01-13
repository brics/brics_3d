/**
 * @file 
 * NearestNeighborANN.h
 *
 * @date: Jan 12, 2010
 * @author: sblume
 */

#ifndef NEARESTNEIGHBORANN_H_
#define NEARESTNEIGHBORANN_H_


#include "INearestNeighbor.h"
#include "ann/include/ANN/ANN.h"

namespace BRICS_3D {

/**
 * @ingroup nearestNeighbor
 * @brief Implementation for the nearest neighbor search algorithm with the ANN library.
 *
 *
 * Further information about the ANN library can be found here: http://www.cs.umd.edu/~mount/ANN/
 *
 */
class NearestNeighborANN: public BRICS_3D::INearestNeighbor {
public:

	/**
	 * @brief Standard constructor
	 */
	NearestNeighborANN();

	/**
	 * @brief Standard destructor
	 */
	virtual ~NearestNeighborANN();

	void setData(vector< vector<float> >* data);
	void setData(vector< vector<double> >* data);
	void setData(PointCloud3D* data);

	int findNearestNeigbor(vector<float>* query);
	int findNearestNeigbor(vector<double>* query);
	int findNearestNeigbor(Point3D* query);

private:

	/// number of nearest neighbors
	int k;

	/// error bound
	double eps;

	/// maximum number of data points
	int maxPts;

	/// data points
	ANNpointArray dataPoints;

	/// query point
	ANNpoint queryPoint;

	/// near neighbor indices
	ANNidxArray nnIndex;

	/// near neighbor distances
	ANNdistArray distances;

	/// search structure
	ANNkd_tree* kdTree;

};

}

#endif /* NEARESTNEIGHBORANN_H_ */

/* EOF */
