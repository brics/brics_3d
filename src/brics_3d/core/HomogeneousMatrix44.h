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

#ifndef BRICS_3D_HOMOGENEOUSMATRIX44_H_
#define BRICS_3D_HOMOGENEOUSMATRIX44_H_

#include "IHomogeneousMatrix44.h"
#include <Eigen/Geometry>

#ifdef EIGEN3
	#define EIGEN_ALIGN_MEMORY EIGEN_ALIGN16
	typedef Eigen::Affine3d Transform3d;
#else
	#define EIGEN_ALIGN_MEMORY EIGEN_ALIGN_128
	using Eigen::Transform3d;
	#include <Eigen/LU>
#endif


namespace brics_3d {

/**
 * @brief Simple implementation of IHomogeneousMatrix44 interface.
 *
 * Matrix can be set up either via manual rotation and translation coefficients or an Eigen2 transformation.
 */
class HomogeneousMatrix44: public brics_3d::IHomogeneousMatrix44 {
public:

	/**
	 * @brief Constructor with rotation and translation coefficients.
	 *
	 * Coefficient layout for rotation: <br>
	 * r0 r1 r2 <br>
	 * r3 r4 r5 <br>
	 * r6 r7 r8 <br>
	 *
	 * @param r0 Rotation coefficient 1,1
	 * @param r1 Rotation coefficient 1,2
	 * @param r2 Rotation coefficient 1,3
	 * @param r3 Rotation coefficient 2,1
	 * @param r4 Rotation coefficient 2,2
	 * @param r5 Rotation coefficient 2,3
	 * @param r6 Rotation coefficient 3,1
	 * @param r7 Rotation coefficient 3,2
	 * @param r8 Rotation coefficient 3,3
	 * @param t0 First translation coefficient
	 * @param t1 Second translation coefficient
	 * @param t2 Third translation coefficient
	 *
	 * Default values are defined as the Identity matrix.
	 */
	HomogeneousMatrix44(double r0 = 1, double r1 = 0, double r2 = 0, double r3 = 0, double r4 = 1, double r5 = 0, double r6 = 0, double r7 = 0, double r8 = 1, double t0 = 0, double t1 = 0, double t2 = 0);

	/**
	 * @brief Constructor with Eigen2 transformation
	 *
	 * @param homogeneousTransformation Transformation defined as Eigen2 transformation.
	 * See also: http://eigen.tuxfamily.org/dox/TutorialGeometry.html
	 *
	 */
	HomogeneousMatrix44(Transform3d *homogeneousTransformation);

	/**
	 * @brief Standard destructor
	 */
	virtual ~HomogeneousMatrix44();

	const double* getRawData() const;

	double* setRawData();

	IHomogeneousMatrix44* operator*(const IHomogeneousMatrix44 &matrix);

	IHomogeneousMatrix44* operator*=(const IHomogeneousMatrix44 &matrix); //TODO: implement?

	IHomogeneousMatrix44* operator=(const IHomogeneousMatrix44 &matrix);

	bool isIdentity(double precision = 0.00001);

	void inverse();

	friend ostream& operator<<(ostream &outStream, const IHomogeneousMatrix44 &matrix);

	/* Some helper functions for conversions: */

	/**
	 * Conversion between x,y,z, roll, pitch, yaw (as defined for planes) to a matrix.
	 *
	 * Transformation is defined as fixes angles rotation in X-Y-Z order.
	 * Cf. Craig, John J.: Introduction to robotics: mechanics and control, 2005.
	 *
	 * NOTE: Roll, Pitch, Yaw definitions might differ. Eg. Richard P. Paul: Robot Manipulators: Mathematics, Programming, and Control, 1981 pp. 45
	 * uses: roll around z, pitch around y and yaw around x.
	 *
	 * @param x X coordinate.
	 * @param y Y coordinate.
	 * @param z R coordinate.
	 * @param roll Rotation around X axis.
	 * @param pitch Roatation around Y axis.
	 * @param yaw Rotation around Z axis.
	 * @param resultMatrix
	 */
	static void xyzRollPitchYawToMatrix(double x, double y, double z, double roll, double pitch, double yaw, IHomogeneousMatrix44::IHomogeneousMatrix44Ptr& resultMatrix);
	static void matrixToXyzRollPitchYaw(IHomogeneousMatrix44::IHomogeneousMatrix44Ptr matrix, double& x, double& y, double& z, double& roll, double& pitch, double& yaw);

	static void quaternionToMatrix(double x, double y, double z, double qx, double qy, double qz, double qw, IHomogeneousMatrix44::IHomogeneousMatrix44Ptr& resultMatrix);
	static void matrixToQuaternion(IHomogeneousMatrix44::IHomogeneousMatrix44Ptr matrix, double& x, double& y, double& z, double& qx, double& qy, double& qz, double& qw);


//	static void xyzRollPitchYawToMatrixPaul(double x, double y, double z, double roll, double pitch, double yaw, IHomogeneousMatrix44::IHomogeneousMatrix44Ptr& resultMatrix);
//	static void matrixToXyzRollPitchYawPaul(IHomogeneousMatrix44::IHomogeneousMatrix44Ptr matrix, double& x, double& y, double& z, double& roll, double& pitch, double& yaw);

	// Calculates distance from origin to point
	static void matrixToDistance(IHomogeneousMatrix44::IHomogeneousMatrix44Ptr matrix, double distance);

private:

	/// Amount of elements in 4x4 matrix
	const static int matrixElements = 16; // 4x4

	/// Array that holds data in column-row (column-major) order
	double matrixData[matrixElements];
};

}

#endif /* BRICS_3D_HOMOGENEOUSMATRIX44_H_ */

/* EOF */
