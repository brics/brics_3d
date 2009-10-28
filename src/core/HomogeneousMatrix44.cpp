/**
 * @file 
 * HomogeneousMatrix44.cpp
 *
 * @date: Oct 23, 2009
 * @author: sblume
 */

#include "HomogeneousMatrix44.h"

namespace brics {

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

HomogeneousMatrix44::HomogeneousMatrix44(Eigen::Transform3d *homogeneousTransformation) {
	double *tmpMatrix;

	tmpMatrix = homogeneousTransformation->data(); //get data in column-row order
	memcpy(&matrixData, tmpMatrix, sizeof(double)*matrixElements);
}

void HomogeneousMatrix44::getRawData(double *matrixBuffer) {
	matrixBuffer = (double*)&matrixData;
}



}
/* EOF */
