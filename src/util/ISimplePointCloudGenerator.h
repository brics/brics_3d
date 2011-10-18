/*
 * ISimplePointCloudGenerator.h
 *
 *  Created on: Oct 5, 2011
 *      Author: Pinaki Sunil Banerjee
 */

#ifndef ISIMPLEPOINTCLOUDGENERATOR_H_
#define ISIMPLEPOINTCLOUDGENERATOR_H_
#include "core/PointCloud3D.h"

namespace BRICS_3D {

class ISimplePointCloudGenerator {
public:


	/**
	 * Generates a 3D model of a simple shape
	 * @param generatedPointCloud Pointcloud representing the requested 3D model
	 */
	virtual void generatePointCloud(BRICS_3D::PointCloud3D *generatedPointCloud)=0;

};

}

#endif /* ISIMPLEPOINTCLOUDGENERATOR_H_ */
