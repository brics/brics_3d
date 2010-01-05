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

class NearestNeighborFLANN : public INearestNeighbor {
public:
	NearestNeighborFLANN();
	virtual ~NearestNeighborFLANN();

	void setData(vector< vector<float> >* data);
	void setData(vector< vector<double> >* data);
	void setData(PointCloud3D* data);

	int findNearestNeigbor(vector<float>* query);
	int findNearestNeigbor(vector<double>* query);
	int findNearestNeigbor(Point3D* query);

private:

	///matrix in major-row representation
	float* dataMatrix;

	int rows;

	int cols;

	FLANNParameters p;

	FLANN_INDEX index_id;

	float speedup;
};

}

#endif /* NEARESTNEIGHBORFLANN_H_ */

/* EOF */
