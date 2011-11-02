/******************************************************************************
* BRICS_3D - 3D Perception and Modeling Library
* Copyright (c) 2011, GPS GmbH
*
* Author: Pinaki Sunil Banerjee
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
