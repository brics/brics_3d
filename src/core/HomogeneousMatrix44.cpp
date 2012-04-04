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

#include "HomogeneousMatrix44.h"

namespace BRICS_3D {

HomogeneousMatrix44::~HomogeneousMatrix44() {

}


HomogeneousMatrix44::HomogeneousMatrix44(double r0, double r1, double r2, double r3, double r4, double r5, double r6, double r7, double r8, double t0, double t1, double t2) {
	/*
	 * column-row layout:
	 * 0 4 8  12
	 * 1 5 9  13
	 * 2 6 10 14
	 * 3 7 11 15
	 */
	/* rotation */
	matrixData[0] = r0;
	matrixData[4] = r1;
	matrixData[8] = r2;
	matrixData[1] = r3;
	matrixData[5] = r4;
	matrixData[9] = r5;
	matrixData[2] = r6;
	matrixData[6] = r7;
	matrixData[10] = r8;

	/* translation */
	matrixData[12] = t0;
	matrixData[13] = t1;
	matrixData[14] = t2;

	//homogeneous coefficients
	matrixData[3] = 0.0;
	matrixData[7] = 0.0;
	matrixData[11] = 0.0;
	matrixData[15] = 1.0;
}

HomogeneousMatrix44::HomogeneousMatrix44(Transform3d *homogeneousTransformation) {
	double *tmpMatrix;

	tmpMatrix = homogeneousTransformation->data(); //get data in column-row order
	memcpy(&matrixData, tmpMatrix, sizeof(double)*matrixElements);
}

const double* HomogeneousMatrix44::getRawData() const {
	return (double*)&matrixData;
}

double* HomogeneousMatrix44::setRawData() {
	return (double*)&matrixData;
}

IHomogeneousMatrix44* HomogeneousMatrix44::operator*(const IHomogeneousMatrix44 &matrix) {
	const double *multiplicand = matrix.getRawData();

	Eigen::Matrix4d tempMatrix1;
	Eigen::Matrix4d tempMatrix2;
	Eigen::Matrix4d result;

	for (int i = 0; i < 16; ++i) { //layout for BRICS and Eigen2 4x4 matrices is the same ;-)
#ifdef EIGEN3
		tempMatrix1(i) = matrixData[i];
		tempMatrix2(i) = multiplicand[i];
#else
		tempMatrix1[i] = matrixData[i];
		tempMatrix2[i] = multiplicand[i];
#endif
	}

	result = tempMatrix1 * tempMatrix2;

	for (int i = 0; i < 16; ++i) { //might be also implemented with memcopy
#ifdef EIGEN3
		matrixData[i] = result(i);
#else
		matrixData[i] = result[i];
#endif
	}

	return this;
}

//IHomogeneousMatrix44* HomogeneousMatrix44::operator*=(const IHomogeneousMatrix44 &matrix1, const IHomogeneousMatrix44 &matrix2) {
//
//}

IHomogeneousMatrix44* HomogeneousMatrix44::operator=(const IHomogeneousMatrix44 &matrix) {
	const double* newMatrixData = matrix.getRawData();

	for (int i = 0; i < matrixElements; ++i) {
		matrixData[i] = newMatrixData[i];
	}

    return this;
}

bool HomogeneousMatrix44::isIdentity(double precision) {
	Eigen::Map<Eigen::Matrix4d> eigenMatrix(matrixData);
	return eigenMatrix.isApprox(Eigen::Matrix4d::Identity(), precision);
}

void HomogeneousMatrix44::inverse() { //could be refactored towards returning a new inverse matrix
	Transform3d result;

	Eigen::Map<Eigen::Matrix4d> eigenMatrix(matrixData);
	Transform3d transform;
	transform.matrix() = eigenMatrix;
	result = transform.inverse();

	double *tmpMatrix;
	tmpMatrix = result.data(); //get data in column-row order
	memcpy(&matrixData, tmpMatrix, sizeof(double)*matrixElements);

//	for (int i = 0; i < 16; ++i) {
//#ifdef EIGEN3
//		matrixData[i] = result.matrix()(i);
//#else
//		matrixData[i] = result[i];
//#endif
//
//	}

}

ostream& operator<<(ostream &outStream, const IHomogeneousMatrix44 &matrix) {
	const double *matrixData = matrix.getRawData();

	/* go through 4x4 column-row layout */
	for (int row = 0; row < 4; ++row) {
		for (int col = row; col < row+9; col += 4) {
			outStream << matrixData[col] << " ";
		}
		outStream << matrixData[row+12]; // without space at end of row
		outStream << std::endl;
	}

	return outStream;
}

}
/* EOF */
