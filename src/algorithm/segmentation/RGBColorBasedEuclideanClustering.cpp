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
#include "util/PCLTypecaster.h"
#include "core/ColorSpaceConvertor.h"

#include <pcl/segmentation/extract_clusters.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include "RGBColorBasedEuclideanClustering.h"

namespace BRICS_3D {

RGBColorBasedEuclideanClustering::RGBColorBasedEuclideanClustering() {
	// TODO Auto-generated constructor stub

}

RGBColorBasedEuclideanClustering::~RGBColorBasedEuclideanClustering() {
	// TODO Auto-generated destructor stub
}


int RGBColorBasedEuclideanClustering::segment(){
	std::cout << "Nothing implemented yet" << std::endl;
}

} /* namespace BRICS_3D */
