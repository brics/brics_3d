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

#ifndef COVARIANCEMATRIX66_H_
#define COVARIANCEMATRIX66_H_

#include "ITransformUncertainty.h"

namespace brics_3d {

class CovarianceMatrix66 : public ITransformUncertainty {
public:
	CovarianceMatrix66();
	CovarianceMatrix66(double x,double y, double z, double roll, double pitch, double yaw);
	virtual ~CovarianceMatrix66();


	int getDimension() const;

	int getRowDimension() const;

	int getColumnDimension() const;

	const double* getRawData() const;

	double* setRawData();

	friend ostream& operator<<(ostream &outStream, const ITransformUncertainty &uncertainty);

private:
	/// Amount of elements in 6x6 matrix
	const static int matrixElements = 36; // 6x6

	/// Row dimansion
	const static int rowDimension = 6;

	/// Column dimension
	const static int columnDimension = 6;

	/// Array that holds data in column-row (column-major) order
	double matrixData[matrixElements];

	/// Set all values to zero.
	void zeros();
};

}

#endif /* COVARIANCEMATRIX66_H_ */

/* EOF */
