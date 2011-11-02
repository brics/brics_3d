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

#ifndef NEARESTNEIGHBORANN_H_
#define NEARESTNEIGHBORANN_H_


#include "algorithm/nearestNeighbor/INearestNeighbor.h"
#include "algorithm/nearestNeighbor/INearestPoint3DNeighbor.h"
#include "algorithm/nearestNeighbor/INearestNeighborSetup.h"
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
class NearestNeighborANN: public INearestNeighbor, public INearestPoint3DNeighbor, public INearestNeighborSetup {
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

	void findNearestNeighbors(vector<float>* query, std::vector<int>* resultIndices, unsigned int k = 1);
	void findNearestNeighbors(vector<double>* query, std::vector<int>* resultIndices, unsigned int k = 1);
	void findNearestNeighbors(Point3D* query, std::vector<int>* resultIndices, unsigned int k = 1);

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
