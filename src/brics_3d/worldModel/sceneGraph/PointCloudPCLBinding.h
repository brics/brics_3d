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

#ifndef RSG_POINTCLOUDPCLBINDING_H_
#define RSG_POINTCLOUDPCLBINDING_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace brics_3d {

namespace rsg {

/**
 * @brief Specialization for PointCloud<pcl::PointXYZ> type to return correct iterator implementation.
 */
template<>
inline IPoint3DIterator::IPoint3DIteratorPtr PointCloud< const pcl::PointCloud<pcl::PointXYZ> >::getPointCloudIterator() {
	PCLPointCloudIterator<pcl::PointXYZ>::PCLPointCloudIteratorPtr it(new brics_3d::PCLPointCloudIterator<pcl::PointXYZ>());
	it->insert(data);
	return it;
}

/**
 * @brief Specialization for PointCloud<pcl::PointXYZRGB> type to return correct iterator implementation.
 */
template<>
inline IPoint3DIterator::IPoint3DIteratorPtr PointCloud< const pcl::PointCloud<pcl::PointXYZRGB> >::getPointCloudIterator() {
	PCLPointCloudIterator<pcl::PointXYZRGB>::PCLPointCloudIteratorPtr it(new brics_3d::PCLPointCloudIterator<pcl::PointXYZRGB>());
	it->insert(data);
	return it;
}

/**
 * @brief Specialization for PointCloud<pcl::PointXYZRGBNormal> type to return correct iterator implementation.
 */
template<>
inline IPoint3DIterator::IPoint3DIteratorPtr PointCloud< const pcl::PointCloud<pcl::PointXYZRGBNormal> >::getPointCloudIterator() {
	PCLPointCloudIterator<pcl::PointXYZRGBNormal>::PCLPointCloudIteratorPtr it(new brics_3d::PCLPointCloudIterator<pcl::PointXYZRGBNormal>());
	it->insert(data);
	return it;
}

/* To be continued... */

}

}

#endif /* RSG_POINTCLOUDPCLBINDING_H_ */

/* EOF */
