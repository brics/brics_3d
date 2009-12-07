/**
 * @file 
 * HomogeneousMatrix44.h
 *
 * @date: Oct 23, 2009
 * @author: sblume
 */

#ifndef HOMOGENEOUSMATRIX44_H_
#define HOMOGENEOUSMATRIX44_H_

#include "IHomogeneousMatrix44.h"
#include <Eigen/Geometry>

namespace BRICS_3D {

/**
 * @brief Simple implementation of IHomogeneousMatrix44 interface.
 *
 * Matrix can be set up either via manual rotation and translation coefficients or an Eigen2 transformation.
 */
class HomogeneousMatrix44: public BRICS_3D::IHomogeneousMatrix44 {
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
	HomogeneousMatrix44(Eigen::Transform3d *homogeneousTransformation);

	const double* getRawData();

	/**
	 * @brief Standard destructor
	 */
	virtual ~HomogeneousMatrix44();

private:

	/// Amount of elements in 4x4 matrix
	const static int matrixElements = 16; // 4x4

	/// Array that holds data in column-row (column-major) order
	double matrixData[matrixElements];
};

}

#endif /* HOMOGENEOUSMATRIX44_H_ */

/* EOF */
