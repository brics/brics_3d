/******************************************************************************
* BRICS_3D - 3D Perception and Modeling Library
* Copyright (c) 2013, KU Leuven
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

#ifndef BRICS_3D_ITRANSFORMUNCERTAINTY_H_
#define BRICS_3D_ITRANSFORMUNCERTAINTY_H_

#include <iostream>
#include <boost/shared_ptr.hpp>
using std::ostream;
using std::istream;

namespace brics_3d {

/**
 * @brief Abstract interface for uncertainty representations for homegenious transformations.
 */
class ITransformUncertainty {
public:

	typedef boost::shared_ptr<ITransformUncertainty> ITransformUncertaintyPtr;
	typedef boost::shared_ptr<ITransformUncertainty const> ITransformUncertaintyConstPtr;

	/**
	 * @brief Default constructor.
	 */
	ITransformUncertainty(){};

	/**
	 * @brief Default destructor.
	 */
	virtual ~ITransformUncertainty(){};

	/**
	 * @brief Return the total dimension of the representation. E.g.
	 * @return The dimension
	 */
	virtual int getDimension() const = 0;

	/**
	 * @brief Return the row dimension of the representation. E.g.
	 * @return The dimension
	 */
	virtual int getRowDimension() const = 0;

	/**
	 * @brief Return the column dimension of the representation. E.g.
	 * @return The dimension
	 */
	virtual int getColumnDimension() const = 0;

	/**
	 * @brief Returns a pointer to a read-only buffer where the raw data is stored.
	 *
	 * In case of matrix data (like a covariance matrix) the buffer is assumbed to be in
	 * column-row order. Similar too @see IHomogeneousMatrix44::getRawData

	 *
	 *
	 * @return (Constant) pointer to buffer where the data is stored.
	 * Buffer has a size of getDimension() double values.
	 */
	virtual const double* getRawData() const = 0;

	/**
	 * @brief Returns a pointer to a writable buffer where the raw data of is stored.
	 */
	virtual double* setRawData() = 0;


	//virtual compound();
	//virtual propagateUncertainty(A,B)
	//virtual fuseUncertainty(A,B)

	/**
	 * Retrieve dimensions for visualization via an ellipsoid.
	 * @param[out] x
	 * @param[out] y
	 * @param[out] z
	 */
	virtual void getVisualizationDimensions(double& x, double& y, double& z) = 0;

	/**
	 * @brief Overridden << operator.
	 *
	 * Writes a homogeneous matrix to a stream e.g. std::cout << homMatObj;
	 *
	 * @param outStream The output stream
	 * @param uncertainty Reference to uncertainty representation which is forwarded to the output stream
	 * @return Output stream
	 */
	friend ostream& operator<<(ostream &outStream, const ITransformUncertainty &uncertainty);

};

}

#endif /* BRICS_3D_ITRANSFORMUNCERTAINTY_H_ */

/* EOF */
