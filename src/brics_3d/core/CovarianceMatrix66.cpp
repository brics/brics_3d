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

#include "CovarianceMatrix66.h"

namespace brics_3d {

CovarianceMatrix66::CovarianceMatrix66() {
	zeros();
}

CovarianceMatrix66::CovarianceMatrix66(double x,double y, double z, double roll, double pitch, double yaw) {
	/*
	 * column-row layout for 6x6 matrix:
	 * 0  6  12  18  24  30
	 * 1  7  13  19  25  31
	 * 2  8  14  20  26  32
	 * 3  8  15  21  27  33
	 * 4  10 16  22  28  34
	 * 5  11 17  23  29  35
	 */

	zeros();
	matrixData[0]  = x;
	matrixData[7]  = y;
	matrixData[14] = z;
	matrixData[21] = roll;
	matrixData[28] = pitch;
	matrixData[35] = yaw;
}

CovarianceMatrix66::~CovarianceMatrix66() {

}


int CovarianceMatrix66::getDimension() const {
	return matrixElements;
}

int CovarianceMatrix66::getRowDimension() const {
	return rowDimension;
}

int CovarianceMatrix66::getColumnDimension() const {
	return columnDimension;
}

const double* CovarianceMatrix66::getRawData() const {
	return (double*)&matrixData;
}

double* CovarianceMatrix66::setRawData() {
	return (double*)&matrixData;
}

void CovarianceMatrix66::zeros() {
	for (int i = 0; i < matrixElements; ++i) {
		matrixData[i] = 0.0;
	}
}

ostream& operator<<(ostream &outStream, const ITransformUncertainty &uncertainty) {
	const double *matrixData = uncertainty.getRawData();

	/* go through mxn column-row layout */
	for (int row = 0; row < uncertainty.getRowDimension(); ++row) {
		for (int col = row; col < row + (uncertainty.getColumnDimension() * (uncertainty.getColumnDimension()-1)); col += uncertainty.getColumnDimension()) {
			outStream << matrixData[col] << " ";
		}
		outStream << matrixData[row + (uncertainty.getColumnDimension() * (uncertainty.getColumnDimension()-1))]; // without space at end of row
		outStream << std::endl;
	}

	return outStream;
}


}

/* EOF */
