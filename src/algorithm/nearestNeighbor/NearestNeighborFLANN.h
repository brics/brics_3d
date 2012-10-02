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

#ifndef BRICS_3D_NEARESTNEIGHBORFLANN_H_
#define BRICS_3D_NEARESTNEIGHBORFLANN_H_

#include "algorithm/nearestNeighbor/INearestNeighbor.h"
#include "algorithm/nearestNeighbor/INearestPoint3DNeighbor.h"
#include "algorithm/nearestNeighbor/INearestNeighborSetup.h"
#include "flann/src/cpp/flann.h"

namespace brics_3d {

/**
 * @ingroup nearestNeighbor
 * @brief Implementation for the nearest neighbor search algorithm with the FLANN library.
 *
 */
class NearestNeighborFLANN : public INearestNeighbor, public INearestPoint3DNeighbor, public INearestNeighborSetup {
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

	void findNearestNeighbors(vector<float>* query, std::vector<int>* resultIndices, unsigned int k = 1);
	void findNearestNeighbors(vector<double>* query, std::vector<int>* resultIndices, unsigned int k = 1);
	void findNearestNeighbors(Point3D* query, std::vector<int>* resultIndices, unsigned int k = 1);

	FLANNParameters getParameters() const;

	void setParameters(FLANNParameters p);

	float getSpeedup() const;





private:

	/// Matrix in major-row representation
	float* dataMatrix;

	/// Number of rows in the dataMatrix
	int rows;

	/// Number of columns in the dataMatrix
	int cols;

	/// Container for various algorithm parameters
	FLANNParameters parameters;

	/// Handle for the reprocessed data (e.g. k-d tree)
	FLANN_INDEX index_id;

	/// Estimated speedup of used algorithm with respect to a brute force approach
	float speedup;
};

}

#endif /* BRICS_3D_NEARESTNEIGHBORFLANN_H_ */

/* EOF */
