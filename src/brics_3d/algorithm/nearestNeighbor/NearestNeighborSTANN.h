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
#ifndef BRICS_3D_NEARESTNEIGHBORSTANN_H_
#define BRICS_3D_NEARESTNEIGHBORSTANN_H_

#include "INearestNeighbor.h"
#include "brics_3d/algorithm/nearestNeighbor/INearestPoint3DNeighbor.h"
#include "brics_3d/algorithm/nearestNeighbor/INearestNeighborSetup.h"
#include "stann/include/dpoint.hpp"
#include "stann/include/sfcnn.hpp"


namespace brics_3d {

/**
 * @brief Dimension of generic k Nearest Neighbor search.
 *
 * <b>IMPORTANT:</b>
 * This parameter <b>needs to be edited</b>, to use arbitrary dimensions in the
 * INearestNeighbor interface.<br>
 * The STANN wrapper class cannot determine the dimension dynamically
 * and thus needs the variable to be set correctly during compile time.
 * If the dimension of the input data does not match this constant,
 * then an exception is thrown.
 *
 */
const unsigned int STANNDimension = 128u;

/// Dimension of 3D Point used for STANN library
const unsigned int STANNPoint3DDimension = 3u;

/// Typedef for the optimized 3D Point representation in the STANN library.
typedef reviver::dpoint<double, STANNPoint3DDimension> STANNPoint3D;

/// Typedef for the Point representation with arbitrary dimension in the STANN library.
typedef reviver::dpoint<double, STANNDimension> STANNPoint;

/**
 * @ingroup nearestNeighbor
 * @brief Implementation for the nearest neighbor search algorithm with the STANN library.
 *
 *
 * Further information about the STANN library can be found here: http://sites.google.com/a/compgeom.com/stann/Home
 *
 * <b>IMPORTANT NOTE: Currently this wrapper only supports dimensionality of 3 and the one defined by the STANNDimension constant!</b>
 *
 */
class NearestNeighborSTANN: public brics_3d::INearestNeighbor, public INearestPoint3DNeighbor, public INearestNeighborSetup {
public:

	/**
	 * @brief Standard constructor
	 */
	NearestNeighborSTANN();

	/**
	 * @brief Standard destructor
	 */
	virtual ~NearestNeighborSTANN();

	void setData(vector< vector<float> >* data);
	void setData(vector< vector<double> >* data);
	void setData(PointCloud3D* data);

	void findNearestNeighbors(vector<float>* query, std::vector<int>* resultIndices, unsigned int k = 1);
	void findNearestNeighbors(vector<double>* query, std::vector<int>* resultIndices, unsigned int k = 1);
	void findNearestNeighbors(Point3D* query, std::vector<int>* resultIndices, unsigned int k = 1);

protected:

	/// Handle to the STANN data representation for 3D points (Morton ordering)
	sfcnn<STANNPoint3D, STANNPoint3DDimension, double>* nearestPoint3DNeigborHandle;

	/// Handle to the STANN data representation (Morton ordering)
	sfcnn<STANNPoint, STANNDimension, double>* nearestNeigborHandle;

	/// Point 3D data in STANN-like format
	vector<STANNPoint3D>* points3D;

	/// Point data in STANN-like format for arbitrary dimensions
	vector<STANNPoint>* points;

	/// Vector with the resulting indices (should be only one index)
	vector<long unsigned int>* resultIndices;

	/// Vector with the resulting distances (should be only one distance). STANN search method returns this as the squared distance.
	vector <double>* squaredResultDistances;

};

}

#endif /* BRICS_3D_NEARESTNEIGHBORSTANN_H_ */

/* EOF */
