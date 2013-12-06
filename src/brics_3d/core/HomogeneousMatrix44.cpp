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

namespace brics_3d {

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
	matrixData[matrixEntry::r11] = r0;
	matrixData[matrixEntry::r12] = r1;
	matrixData[matrixEntry::r13] = r2;
	matrixData[matrixEntry::r21] = r3;
	matrixData[matrixEntry::r22] = r4;
	matrixData[matrixEntry::r23] = r5;
	matrixData[matrixEntry::r31] = r6;
	matrixData[matrixEntry::r32] = r7;
	matrixData[matrixEntry::r33] = r8;

	/* translation */
	matrixData[matrixEntry::x] = t0;
	matrixData[matrixEntry::y] = t1;
	matrixData[matrixEntry::z] = t2;

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
	const double *tmpMultiplicand = matrix.getRawData();
	double multiplicand[matrixElements]; // this wil be interpreted(mapped) as an Eigen matrix, thus we need sth writable...
	memcpy(&multiplicand, tmpMultiplicand, sizeof(double)*matrixElements);

	Eigen::Map<Eigen::Matrix4d> tempMatrix1(matrixData); //layout for BRICS and Eigen2 4x4 matrices is the same ;-)
	Eigen::Map<Eigen::Matrix4d> tempMatrix2(multiplicand);
	Eigen::Matrix4d result;

	result = tempMatrix1 * tempMatrix2;

	/* copy back the result */
	double *tmpMatrix;
	tmpMatrix = result.data(); //get data in column-row order
	memcpy(&matrixData, tmpMatrix, sizeof(double)*matrixElements);

	return this;
}

//IHomogeneousMatrix44* HomogeneousMatrix44::operator*=(const IHomogeneousMatrix44 &matrix1, const IHomogeneousMatrix44 &matrix2) {
//
//}

IHomogeneousMatrix44* HomogeneousMatrix44::operator=(const IHomogeneousMatrix44 &matrix) {
	const double* newMatrixData = matrix.getRawData();
	memcpy(&matrixData, newMatrixData, sizeof(double)*matrixElements);

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

void HomogeneousMatrix44::xyzRollPitchYawToMatrix(double x, double y, double z, double roll, double pitch, double yaw, IHomogeneousMatrix44::IHomogeneousMatrix44Ptr& resultMatrix) {
	double* matrixData = resultMatrix->setRawData();

	/* Translation */
	matrixData[matrixEntry::x] = x;
	matrixData[matrixEntry::y] = y;
	matrixData[matrixEntry::z] = z;

	/* Rotation */
	/*
	 * column-row layout:
	 * 0 4 8  12
	 * 1 5 9  13
	 * 2 6 10 14
	 * 3 7 11 15
     *
     *  <=>
     *
	 * r11 r12 r13  12
	 * r21 r22 r23  13
	 * r31 r32 r33  14
	 * 3    7   11  15
	 */
	matrixData[matrixEntry::r11] = cos(yaw)*cos(pitch);
	matrixData[matrixEntry::r12] = cos(yaw)*sin(pitch)*sin(roll) - sin(yaw)*cos(roll);
	matrixData[matrixEntry::r13] = cos(yaw)*sin(pitch)*cos(roll) + sin(yaw)*sin(roll);

	matrixData[matrixEntry::r21] = sin(yaw)*cos(pitch);
	matrixData[matrixEntry::r22] = sin(yaw)*sin(pitch)*sin(roll) + cos(yaw)*cos(roll);
	matrixData[matrixEntry::r23] = sin(yaw)*sin(pitch)*cos(roll) - cos(yaw)*sin(roll);

	matrixData[matrixEntry::r31] = -sin(pitch);
	matrixData[matrixEntry::r32] = cos(pitch)*sin(roll);
	matrixData[matrixEntry::r33] = cos(pitch)*cos(roll);
}


void HomogeneousMatrix44::matrixToXyzRollPitchYaw(IHomogeneousMatrix44::IHomogeneousMatrix44Ptr matrix, double& x, double& y, double& z, double& roll, double& pitch, double& yaw) {
	const double* matrixData = matrix->getRawData();

	x = matrixData[matrixEntry::x];
	y = matrixData[matrixEntry::y];
	z = matrixData[matrixEntry::z];

	/*
	 * column-row layout:
	 * 0 4 8  12
	 * 1 5 9  13
	 * 2 6 10 14
	 * 3 7 11 15
     *
     *  <=>
     *
	 * r11 r12 r13  12
	 * r21 r22 r23  13
	 * r31 r32 r33  14
	 * 3    7   11  15
	 */

	//cf. Craig pp. 43
	// NOTE: Numerical instabilitiy for pitch around  PI/2 and -PI/2.
	pitch = atan2(-matrixData[matrixEntry::r31], sqrt (matrixData[matrixEntry::r11]*matrixData[matrixEntry::r11] + matrixData[matrixEntry::r21]*matrixData[matrixEntry::r21]) ); // <- correct? (should be 2x more?)

	double c_pitch = cos(pitch);
	double tolerance = 10e-5;
	if ( fabs(c_pitch - M_PI*0.5) < tolerance) {
//		LOG(WARN) << "Gimbal Lock detected.";
		if (pitch > 0) { // case + PI/2
			pitch = M_PI * 0.5;
			yaw = 0;
			roll = atan2(matrixData[matrixEntry::r21], matrixData[matrixEntry::r22]);
		} else { // case - PI/2
			pitch = -M_PI * 0.5;
			yaw = 0;
			roll = -atan2(matrixData[matrixEntry::r21], matrixData[matrixEntry::r22]);
		}
		return;
	}

	roll = atan2(matrixData[matrixEntry::r32]/c_pitch, matrixData[matrixEntry::r33]/c_pitch);
	yaw = atan2(matrixData[matrixEntry::r21]/c_pitch, matrixData[matrixEntry::r11]/c_pitch);



}

//void HomogeneousMatrix44::xyzRollPitchYawToMatrixPaul(double x, double y, double z, double roll, double pitch, double yaw, IHomogeneousMatrix44::IHomogeneousMatrix44Ptr& resultMatrix) {
//	double* matrixData = resultMatrix->setRawData();
//
//	/* Translation */
//	matrixData[12] = x;
//	matrixData[13] = y;
//	matrixData[14] = z;
//
//	/* Rotation */
//	/*
//	 * column-row layout:
//	 * 0 4 8  12
//	 * 1 5 9  13
//	 * 2 6 10 14
//	 * 3 7 11 15
//     *
//     *  <=>
//     *
//	 * r11 r12 r13  12
//	 * r21 r22 r23  13
//	 * r31 r32 r33  14
//	 * 3    7   11  15
//	 */
//	matrixData[0] = cos(roll)*cos(pitch);
//	matrixData[4] = cos(roll)*sin(pitch)*sin(yaw) - sin(roll)*cos(yaw);
//	matrixData[8] = cos(roll)*sin(pitch)*cos(yaw) + sin(roll)*sin(yaw);
//
//	matrixData[1] = sin(roll)*cos(pitch);
//	matrixData[5] = sin(roll)*sin(pitch)*sin(yaw) + cos(roll)*cos(yaw);
//	matrixData[9] = sin(roll)*sin(pitch)*cos(yaw) - cos(roll)*sin(yaw);
//
//	matrixData[2] = -sin(pitch);
//	matrixData[6] = cos(pitch)*sin(yaw);
//	matrixData[10] = cos(pitch)*cos(yaw);
//}
//
//void HomogeneousMatrix44::matrixToXyzRollPitchYawPaul(IHomogeneousMatrix44::IHomogeneousMatrix44Ptr matrix, double& x, double& y, double& z, double& roll, double& pitch, double& yaw) {
//	const double* matrixData = matrix->getRawData();
//
//	x = matrixData[12];
//	y = matrixData[13];
//	z = matrixData[14];
//
//	/*
//	 * column-row layout:
//	 * 0 4 8  12
//	 * 1 5 9  13
//	 * 2 6 10 14
//	 * 3 7 11 15
//     *
//     *  <=>
//     *
//	 * r11 r12 r13  12
//	 * r21 r22 r23  13
//	 * r31 r32 r33  14
//	 * 3    7   11  15
//	 */
//
//	pitch = atan2(-matrixData[3], sqrt(matrixData[0]*matrixData[0] + matrixData[1]*matrixData[1]));
//	yaw = atan2(matrixData[1]/cos(pitch), matrixData[0]/cos(pitch));
//	roll = atan2(matrixData[6]/cos(pitch), matrixData[10]/cos(pitch));
//}

void HomogeneousMatrix44::quaternionToMatrix(double x, double y, double z, double qx, double qy, double qz, double qw, IHomogeneousMatrix44::IHomogeneousMatrix44Ptr& resultMatrix) {
		double* matrixData = resultMatrix->setRawData();

		/* Translation */
		matrixData[matrixEntry::x] = x;
		matrixData[matrixEntry::y] = y;
		matrixData[matrixEntry::z] = z;

		// cf. Handbook of robotics, 2008
		// e0 = qw
		// e1 = qx
		// e2 = qy
		// e3 = qz

		/* Rotation */
		//r11 r12 r13
		matrixData[matrixEntry::r11] = 1 - 2*(qy*qy + qz*qz);
		matrixData[matrixEntry::r12] = 2  *  (qx*qy - qw*qz);
		matrixData[matrixEntry::r13] = 2  *  (qx*qz + qw*qy);

		//r21 r22 r23
		matrixData[matrixEntry::r21] = 2  *  (qx*qy + qw*qz);
		matrixData[matrixEntry::r22] = 1 - 2*(qx*qx + qz*qz);
		matrixData[matrixEntry::r23] = 2  *  (qy*qz - qw*qx);

		//r31 r32 r33
		matrixData[matrixEntry::r31] = 2  *  (qx*qz - qw*qy);
		matrixData[matrixEntry::r32] = 2  *  (qy*qz + qw*qx);
		matrixData[matrixEntry::r33] = 1 - 2*(qx*qx + qy*qy);
}

void HomogeneousMatrix44::matrixToQuaternion(IHomogeneousMatrix44::IHomogeneousMatrix44Ptr matrix, double& x, double& y, double& z, double& qx, double& qy, double& qz, double& qw) {
	const double* matrixData = matrix->getRawData();

	/* Translation */
	x = matrixData[matrixEntry::x];
	y = matrixData[matrixEntry::y];
	z = matrixData[matrixEntry::z];

	/* Rotation */
	qw = 1/2.0 * (sqrt(1 + matrixData[matrixEntry::r11] + matrixData[matrixEntry::r22] + matrixData[matrixEntry::r33]));
	assert (qw != 0.0);
	qx = (matrixData[matrixEntry::r32] - matrixData[matrixEntry::r23]) / 4 * qw;
	qy = (matrixData[matrixEntry::r13] - matrixData[matrixEntry::r31]) / 4 * qw;
	qz = (matrixData[matrixEntry::r21] - matrixData[matrixEntry::r12]) / 4 * qw;

}


}
/* EOF */
