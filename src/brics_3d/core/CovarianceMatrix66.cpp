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
#include "Logger.h"
#include "HomogeneousMatrix44.h"
#include <Eigen/Dense>

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

void CovarianceMatrix66::getVisualizationDimensions(double& x, double& y, double& z) {
	x = 0;
	y = 0;
	z = 0;
	int dimension = 1;//3;

	Eigen::Map<Eigen::Matrix<double, 6, 6> > covariance(matrixData); //layout for BRICS and Eigen 6x6 matrices is the same

	/* compute eigen vectors and values */
	Eigen::MatrixXd eigenvectors;
	Eigen::VectorXd eigenvalues;
//	Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6> > evd (covariance);
	Eigen::EigenSolver<Eigen::Matrix<double, 6, 6> > evd (covariance);
	eigenvalues = evd.eigenvalues().real (); //NOTE eigen values not ordered by absolute values!
	eigenvectors = evd.eigenvectors().real ();

	// For N standard deviations spread of data, the radii of the eliipsoid will
	// be given by N*SQRT(eigenvalues).


//	LOG(DEBUG) << "\t  Covariance Matrix:";
//	LOG(DEBUG) << std::endl << *this;
//	LOG(DEBUG) << "\t Eigen values + vectors";
	LOG(DEBUG) << std::endl << eigenvalues ;
	LOG(DEBUG) << std::endl << eigenvectors;

#if 0
	/* unfortunately there is no usable order in the results,
	 * but there is hope:
	 */
	Eigen::Matrix<double, 6, 1> indexMask;//(0,1,2,3,4,5);
	Eigen::Matrix<double, 1, 1> resultIndex;
	indexMask(0) = 0;
	indexMask(1) = 1;
	indexMask(2) = 2;
	indexMask(3) = 3;
	indexMask(4) = 4;
	indexMask(5) = 5;

	resultIndex = eigenvectors.row(0) * indexMask;
	LOG(DEBUG) << "IDX for 0 :" << resultIndex(0);
	int indexX = static_cast<int>(resultIndex(0));
	resultIndex = eigenvectors.row(1) * indexMask;
	LOG(DEBUG) << "IDX for 1 :" << resultIndex(0);
	int indexY = static_cast<int>(resultIndex(0));
	resultIndex = eigenvectors.row(2) * indexMask;
	LOG(DEBUG) << "IDX for 2 :" << resultIndex(0);
	int indexZ = static_cast<int>(resultIndex(0));
#endif

	int indexX = 0;
	int indexY = 1;
	int indexZ = 2;

	x = dimension * sqrt(eigenvalues(indexX));
	y = dimension * sqrt(eigenvalues(indexY));
	z = dimension * sqrt(eigenvalues(indexZ));

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


extern CovarianceMatrix66::CovarianceMatrix66Ptr compoundCovariance(IHomogeneousMatrix44::IHomogeneousMatrix44Ptr mean1, CovarianceMatrix66::CovarianceMatrix66Ptr uncertainty1,
		                                                            IHomogeneousMatrix44::IHomogeneousMatrix44Ptr mean2, CovarianceMatrix66::CovarianceMatrix66Ptr uncertainty2) {
	assert(mean1 != 0);
	assert(uncertainty1 != 0);
	assert(mean2 != 0);
	assert(uncertainty2 != 0);

	CovarianceMatrix66::CovarianceMatrix66Ptr result(new CovarianceMatrix66());
	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr resultingMean(new HomogeneousMatrix44());
	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr mean1Temp(new HomogeneousMatrix44());
	double* matrixData;

	Eigen::Matrix3d M;
	Eigen::Matrix3d K1;
	Eigen::Matrix3d K2;
	Eigen::Matrix3d R1;
	Eigen::Matrix3d zeros = Eigen::Matrix3d::Zero();
	Eigen::Matrix3d identity = Eigen::Matrix3d::Identity();
	Eigen::Matrix<double, 6, 12> jacobian;
	Eigen::Matrix<double, 12, 12> tmpAggregatedCovariances;
	Eigen::Matrix<double, 6, 6> resultCovariance;

	/*
	 *
	 * Smith & Cheeseman rotation Matrix coneention:
	 * nx ox ax
	 * ny oy ay
	 * nz oz az
	 *
	 *   <=>
	 *
	 * 0 4 8  12
	 * 1 5 9  13
	 * 2 6 10 14
	 * 3 7 11 15
	 */
	matrixData = mean1->setRawData();
	double x1;// = matrixData[12];
	double y1;// = matrixData[13];
	double z1;// = matrixData[14];
//	double nx1 = matrixData[0];
	double ox1 = matrixData[4];
	double ax1 = matrixData[8];
//	double ny1 = matrixData[1];
	double oy1 = matrixData[5];
	double ay1 = matrixData[9];
//	double nz1 = matrixData[2];
	double oz1 = matrixData[6];
	double az1 = matrixData[10];
	double roll1, pitch1, yaw1;
	HomogeneousMatrix44::matrixToXyzRollPitchYaw(mean1, x1, y1, z1, roll1, pitch1, yaw1);


	matrixData = mean2->setRawData();
	double x2;// = matrixData[12];
	double y2;// = matrixData[13];
	double z2;// = matrixData[14];
//	double nx2 = matrixData[0];
	double ox2 = matrixData[4];
	double ax2 = matrixData[8];
//	double ny2 = matrixData[1];
//	double oy2 = matrixData[5];
//	double ay2 = matrixData[9];
//	double nz2 = matrixData[2];
//	double oz2 = matrixData[6];
//	double az2 = matrixData[10];
	double roll2, pitch2, yaw2;
//	mean2->getRollPitchYaw(roll2, pitch2, yaw2);
	HomogeneousMatrix44::matrixToXyzRollPitchYaw(mean2, x2, y2, z2, roll2, pitch2, yaw2);


	*mean1Temp = *mean1;
	(*resultingMean) = *( (*mean1Temp) * (*mean2) );

	matrixData = resultingMean->setRawData();
	double x3;// = matrixData[12];
	double y3;// = matrixData[13];
	double z3;// = matrixData[14];
	double roll3, pitch3, yaw3;
//	resultingMean->getRollPitchYaw(roll3, pitch3, yaw3);
	HomogeneousMatrix44::matrixToXyzRollPitchYaw(resultingMean, x3, y3, z3, roll3, pitch3, yaw3);


	matrixData = uncertainty1->setRawData();
	Eigen::Map<Eigen::Matrix<double, 6, 6> > covariance_ij(matrixData);

	matrixData = uncertainty2->setRawData();
	Eigen::Map<Eigen::Matrix<double, 6, 6> > covariance_jk(matrixData);

	matrixData = mean1->setRawData();
	Eigen::Map<Eigen::Matrix<double, 4, 4> > mean1Matrix(matrixData);
	R1 = mean1Matrix.block<3,3>(0,0); // extract rotational part

	/*
	 * M:
	 * −(y3 − y1) 	(z3 − z1) cos 	1 ox1x2 − nx1y2
	 * x3 − x1 		(z3 − z1) sin 	1 oy1x2 − ny1y2
	 * 0 −x2 cos 1 cos  1 + y2 cos 1 sin  1 − z2 sin 1 oz1x2 − nz1y2
	 *
	 */
	M(0,0) = -(y3 - y1);
	M(0,1) =  (z3 - z1) * cos(roll1);
	M(0,2) =  (ax1*y2 - ox1*z2);
	M(1,0) =  (x3 -x1);
	M(1,1) =  (z3 - z1) * sin(roll1);
	M(1,2) =  (ay1*y2 - oy1*z2);
	M(2,0) =  0;
	M(2,1) =  (x2*cos(pitch1)) - (y2 * sin(pitch1) * sin(yaw1)) - (z2*sin(roll1)*cos(yaw1));
	M(2,2) =  az1*y2 - oz1*z2;

//	1 [sin 3 sin(3 − 1)]/ cos 3 [ox2 sin  3 + ax2 cos  3]/ cos 3
//	0 cos(3 − 1) −cos 1 sin(3 − 1)
//	0 [sin(3 − 1)]/ cos 3 [cos 1 cos(3 − 1)]/ cos 3

	K1(0,0) = 1;
	K1(0,1) = (sin(pitch3) * sin(roll3 - roll1)) / cos(pitch3);
	K1(0,2) = ((ox2 * sin(yaw3)) + (ax2 * cos(yaw3)))/ cos(pitch3);
	K1(1,0) = 0;
	K1(1,1) = cos(roll3 -roll1);
	K1(1,2) = -(cos(pitch1) * sin(roll3 - roll1));
	K1(2,0) = 0;
	K1(2,1) = (sin(roll3 -roll1)) / cos(pitch3);
	K1(2,2) = ((cos(pitch1) *  cos(roll3 - roll1))/ cos(pitch3));

//	[cos 2 cos( 3 −  2)]/ cos 3 [sin( 3 −  2)]/ cos 3 0
//	−cos 2 sin( 3 −  2) cos( 3 −  2) 0
//	[ax1 cos 3 + ay1 sin 3]/ cos 3 [sin 3 sin( 3 −  2)]/ cos 3 1

	K2(0,0) = (cos(pitch2) * cos(yaw3 - yaw2)) / cos(pitch3);
	K2(0,1) = (sin(yaw3 -  yaw2))/ cos (pitch3);
	K2(0,2) = 0;
	K2(1,0) = -(cos(pitch2) * sin(yaw3 - yaw2));
	K2(1,1) = cos(yaw3 -yaw2);
	K2(1,2) = 0;
	K2(2,0) = (ax1 * cos(roll3) + ay1 * sin(roll3)) / cos(pitch3);
	K2(2,1) = (sin(pitch3) * sin(yaw3 -  yaw2))/ cos(pitch3);

	/*
	 * assamble the jacobian:
	 *  I3×3 M R1 03×3
	 *  03×3 K1 03×3 K2
	 */
	jacobian.block<3,3>(0,0) = identity;
	jacobian.block<3,3>(0,3) = M;
	jacobian.block<3,3>(0,6) = R1;
	jacobian.block<3,3>(0,9) = zeros;
	jacobian.block<3,3>(3,0) = zeros;
	jacobian.block<3,3>(3,3) = K1;
	jacobian.block<3,3>(3,6) = zeros;
	jacobian.block<3,3>(3,9) = K2;

	/*
	 * Here we imlicitly assume no cross coralations between the two covariances.
	 * thus we set the remaunter of the matrix to zero
	 */
	tmpAggregatedCovariances = Eigen::Matrix<double, 12, 12>::Zero();
	tmpAggregatedCovariances.block<6,6>(0,0) = covariance_ij;
	tmpAggregatedCovariances.block<6,6>(6,6) = covariance_jk;
	LOG(DEBUG) << "Compount tmpAggregatedCovariances:" << std::endl << tmpAggregatedCovariances;


	/* It is essentially:
	 * C_Y = F_X * C_X * F_X^T
	 *
	 * F_X is the jacobian matrix as defined in  \cite{smith90estimating}
	 * pp. 26
	 */
	resultCovariance = jacobian * tmpAggregatedCovariances * jacobian.transpose();

	matrixData = result->setRawData();
	double *tmpMatrix;
	tmpMatrix = resultCovariance.data(); //get data in column-row order
	for (int i = 0; i < result->getDimension(); ++i) {
		matrixData[i] = resultCovariance(i);
	}

	LOG(DEBUG) << "Compount jacobian:" << std::endl << jacobian;
	LOG(DEBUG) << "Compount resultCovariance covariance:" << std::endl << resultCovariance;
//	LOG(DEBUG) << "Compount result covariance:" << std::endl << *result;

	return result;
}

extern CovarianceMatrix66::CovarianceMatrix66Ptr invertCovariance(IHomogeneousMatrix44::IHomogeneousMatrix44Ptr mean,
		                                                            CovarianceMatrix66::CovarianceMatrix66Ptr uncertainty) {
	assert(mean != 0);
	assert(uncertainty != 0);

	CovarianceMatrix66::CovarianceMatrix66Ptr result(new CovarianceMatrix66());
	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr resultingMean(new HomogeneousMatrix44());
	double* matrixData;

	Eigen::Matrix3d N;
	Eigen::Matrix3d Q;
	Eigen::Matrix3d R;
	Eigen::Matrix3d zeros = Eigen::Matrix3d::Zero();
	Eigen::Matrix<double, 6, 6> jacobian;
	Eigen::Matrix<double, 12, 12> tmpAggregatedCovariances;
	Eigen::Matrix<double, 6, 6> resultCovariance;

	/*
	 *
	 * Smith & Cheeseman rotation Matrix convention:
	 * nx ox ax
	 * ny oy ay
	 * nz oz az
	 *
	 *   <=>
	 *
	 * 0 4 8  12
	 * 1 5 9  13
	 * 2 6 10 14
	 * 3 7 11 15
	 */
	matrixData = mean->setRawData();
	double x;// = matrixData[12];
	double y;// = matrixData[13];
	double z;// = matrixData[14];
	double nx = matrixData[0];
	double ox = matrixData[4];
	double ax = matrixData[8];
	double ny = matrixData[1];
	double oy = matrixData[5];
	double ay = matrixData[9];
	double nz = matrixData[2];
	double oz = matrixData[6];
	double az = matrixData[10];
	double roll, pitch, yaw;
//	mean->getRollPitchYaw(roll, pitch, yaw);
	HomogeneousMatrix44::matrixToXyzRollPitchYaw(mean, x, y, z, roll, pitch, yaw);


	matrixData = uncertainty->setRawData();
	Eigen::Map<Eigen::Matrix<double, 6, 6> > covariance_ij(matrixData);

	matrixData = mean->setRawData();
	Eigen::Map<Eigen::Matrix<double, 4, 4> > meanMatrix(matrixData);
	R = meanMatrix.block<3,3>(0,0); // extract rotational part

//	double x_tick = -(nx*x + ny*y + nz*z);
	double y_tick = -(ox*x + oy*y + oz*z);
	double z_tick = -(ax*x + ay*y + az*z);

	/*
	 *  N =
	 * 	nyx − nxy −nzx cos  − nzy sin  + z cos  0
	 * 	oyx − oxy −ozx cos  − ozy sin + z sin  sin   z0
	 * 	ayx − axy −azx cos  − azy sin  + z sin  cos   −y0
	 */
	N(0,0) = (ny*x) - (nx*y);
	N(0,1) = (-nz*x*cos(roll)) - (nz*y*sin(roll)) + (z*cos(pitch));
	N(0,2) = 0;
	N(1,0) = (oy*x) - (ox*y);
	N(1,1) = (-oz*x*cos(roll)) - (oz*y*sin(roll)) + (z*sin(pitch)*sin(yaw));
	N(1,2) = z_tick;
	N(2,0) = (ay*x) - (ax*y);
	N(2,1) = (-az*x*cos(roll)) - (az*y*sin(roll)) + (z*sin(pitch)*cos(yaw));
	N(2,2) = -y_tick;

	/*
	 * Q=
	 * −az/(1 − ax2) −ay cos /(1 − ax2) nxax/(1 − ax2)
	 * ay/(1 − ax2)1/2 −az cos /(1 − ax2)1/2 ox/(1 − ax2)1/2
	 * azax/(1 − ax2) −ox cos  /(1 − ax2) −nx/(1 − ax2)
	 */
	double ax2 = ax*ax;//?
	double tolerance = 10e-6;
	if( (fabs(1 - ax2) - fabs(1 -ax2)) <  tolerance) {
		LOG(ERROR) << "Division by 0 detected.";
	}
	Q(0,0) = -az / (1-ax2);
	Q(0,1) = (-ay*cos(roll)) / (1 - ax2);
	Q(0,2) = nx*ax / (1 - ax2);
	Q(1,0) = ay / sqrt(1 - ax2);
	Q(1,1) = (-az*cos(roll)) / sqrt(1 - ax2);
	Q(1,2) = ox / sqrt(1 - ax2);
	Q(2,0) = az*ax / (1 - ax2);
	Q(2,1) = -ox*cos(yaw) / (1 - ax2);
	Q(2,2) = -nx / (1 - ax2);

	/*
	 * Assamble the jacobian:
	 *
	 * | −R^T   N |
	 * | 03×3   Q |
	 *
	 */
	jacobian.block<3,3>(0,0) = -R.transpose();
	jacobian.block<3,3>(0,3) = N;
	jacobian.block<3,3>(3,0) = zeros;
	jacobian.block<3,3>(3,3) = Q;

	/*
	 * The actual caomputation based on the jacobian matrix.
	 */
	resultCovariance = jacobian * covariance_ij * jacobian.transpose();

	matrixData = result->setRawData();
	double *tmpMatrix;
	tmpMatrix = resultCovariance.data(); //get data in column-row order
	for (int i = 0; i < result->getDimension(); ++i) {
		matrixData[i] = resultCovariance(i);
	}

	LOG(DEBUG) << "Inversion jacobian:" << std::endl << jacobian;
	LOG(DEBUG) << "Inverted resultCovariance covariance:" << std::endl << resultCovariance;

	return result;
}

extern bool mergeCovariance(IHomogeneousMatrix44::IHomogeneousMatrix44Ptr mean1,
		                    CovarianceMatrix66::CovarianceMatrix66Ptr uncertainty1,
		                    IHomogeneousMatrix44::IHomogeneousMatrix44Ptr mean2,
		                    CovarianceMatrix66::CovarianceMatrix66Ptr uncertainty2,
		                    IHomogeneousMatrix44::IHomogeneousMatrix44Ptr mergedMean,
		                    CovarianceMatrix66::CovarianceMatrix66Ptr mergedUncertainty) {

	assert(mean1 != 0);
	assert(uncertainty1 != 0);
	assert(mean2 != 0);
	assert(uncertainty2 != 0);
	assert(mergedMean != 0);
	assert(mergedUncertainty != 0);

	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr resultingMean(new HomogeneousMatrix44());
	double* matrixData;

	Eigen::Matrix<double, 6, 6> kalmanGain; //K
	Eigen::Matrix<double, 6, 6> resultCovariance;

	matrixData = uncertainty1->setRawData();
	Eigen::Map<Eigen::Matrix<double, 6, 6> > covariance_1(matrixData);

	matrixData = uncertainty2->setRawData();
	Eigen::Map<Eigen::Matrix<double, 6, 6> > covariance_2(matrixData);

	LOG(DEBUG) << "covariance_1 " << std::endl << covariance_1;
	LOG(DEBUG) << "covariance_2 " << std::endl << covariance_2;

	/* calculate the Kalmang gain -> this will give us a ratio whom to trust more */
	Eigen::Matrix<double, 6, 6> addedCovariances = covariance_1 + covariance_2;
	Eigen::Matrix<double, 6, 6> addedCovariancesInverse;
	LOG(DEBUG) << "addedCovariances " << std::endl << addedCovariances;
	Eigen::ColPivHouseholderQR<Eigen::Matrix<double, 6, 6> > tempCovariances(addedCovariances); // ColPivHouseholderQR Speed: +, Accuracy: ++
	addedCovariancesInverse = tempCovariances.inverse();
//	addedCovariancesInverse = addedCovariances.inverse();
	LOG(DEBUG) << "addedCovariancesInverse " << std::endl << addedCovariancesInverse;
	kalmanGain = covariance_1 * addedCovariancesInverse;
	LOG(DEBUG) << "kalmanGain " << std::endl << kalmanGain;

	/* wieght the resulting covariance with he Kalman gain */
	resultCovariance = covariance_1 - (covariance_1 * kalmanGain);

	/* calculate the new mean:
	 * X3 = X1 + K * (X 2- X1)
	 * Note X contains of x,y,z,roll,pitch,yaw -> as in the covariance matrix
	 */
//	matrixData = mean1->setRawData();
//	double x1 = matrixData[12];
//	double y1 = matrixData[13];
//	double z1 = matrixData[14];
	double x1, y1, z1;
	double roll1, pitch1, yaw1;
	HomogeneousMatrix44::matrixToXyzRollPitchYaw(mean1, x1, y1, z1, roll1, pitch1, yaw1);
	//mean1->getRollPitchYaw(roll1, pitch1, yaw1);
	Eigen::Matrix<double, 6, 1> X1;
	X1(0,0) = x1;
	X1(1,0) = y1;
	X1(2,0) = z1;
	X1(3,0) = roll1;
	X1(4,0) = pitch1;
	X1(5,0) = yaw1;

//	matrixData = mean2->setRawData();
//	double x2 = matrixData[12];
//	double y2 = matrixData[13];
//	double z2 = matrixData[14];
//	double roll2, pitch2, yaw2;
//	mean2->getRollPitchYaw(roll2, pitch2, yaw2);
	double x2, y2, z2;
	double roll2, pitch2, yaw2;
	HomogeneousMatrix44::matrixToXyzRollPitchYaw(mean2, x2, y2, z2, roll2, pitch2, yaw2);
	Eigen::Matrix<double, 6, 1> X2;
	X2(0,0) = x2;
	X2(1,0) = y2;
	X2(2,0) = z2;
	X2(3,0) = roll2;
	X2(4,0) = pitch2;
	X2(5,0) = yaw2;

	Eigen::Matrix<double, 6, 1> X3;
	X3 = X1 + kalmanGain * (X2-X1);

	/* prepate output: merged mean */
//	matrixData = mergedMean->setRawData();
//	matrixData[12] = X3(0,0);
//	matrixData[13] = X3(1,0);
//	matrixData[14] = X3(2,0);
	LOG(DEBUG) << "Merged X3: " << std::endl << X3;
	HomogeneousMatrix44::xyzRollPitchYawToMatrix(X3(0,0), X3(1,0), X3(2,0), X3(3,0), X3(4,0), X3(5,0), mergedMean);
//	TODO rotation

	/* prepate output: merged covariance */
	matrixData = mergedUncertainty->setRawData();
	for (int i = 0; i < mergedUncertainty->getDimension(); ++i) {
		matrixData[i] = resultCovariance(i);
	}

	LOG(DEBUG) << "Merged mean: " << std::endl << *mergedMean;
	LOG(DEBUG) << "Merged covariance: " << std::endl << *mergedUncertainty;

	return true;
}

}

/* EOF */
