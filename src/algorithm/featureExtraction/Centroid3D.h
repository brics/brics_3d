/*
 * Centroid3D.h
 *
 *  Created on: Oct 17, 2011
 *      Author: Pinaki Sunil Banerjee
 */

#ifndef CENTROID3DESTIMATION_H_
#define CENTROID3DESTIMATION_H_

#include "core/PointCloud3D.h"
#include "core/ColoredPointCloud3D.h"

#include <Eigen/Dense>
namespace BRICS_3D {

class Centroid3D {
public:
	Centroid3D();
	virtual ~Centroid3D();

	Eigen::Vector3d computeCentroid(BRICS_3D::PointCloud3D *inCloud);

	Eigen::Vector3d computeCentroid(BRICS_3D::ColoredPointCloud3D *inCloud);

};

}

#endif /* CENTROID3DESTIMATION_H_ */
