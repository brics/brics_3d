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

#ifndef ISIMPLEPOINTCLOUDGENERATOR_H_
#define ISIMPLEPOINTCLOUDGENERATOR_H_
#include "core/PointCloud3D.h"

namespace brics_3d {

class ISimplePointCloudGenerator {
public:


	/**
	 * Generates a 3D model of a simple shape
	 * @param generatedPointCloud Pointcloud representing the requested 3D model
	 */
	virtual void generatePointCloud(brics_3d::PointCloud3D *generatedPointCloud)=0;

};

}

#endif /* ISIMPLEPOINTCLOUDGENERATOR_H_ */
