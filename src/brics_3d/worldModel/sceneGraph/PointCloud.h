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

#ifndef RSG_POINTCLOUD_H
#define RSG_POINTCLOUD_H

#include "Shape.h"
#include "brics_3d/core/IPoint3DIterator.h"
#include "brics_3d/core/PointCloud3D.h" //for specialization
#include "brics_3d/core/PointCloud3DIterator.h" //for specialization

namespace brics_3d {

namespace rsg {

/**
 * Abstract interface for point cloud data in the scenegraph.
 * @ingroup sceneGraph
 *
 */
template<typename PointCloudT>
class PointCloud : public Shape {
  public:

	typedef boost::shared_ptr< PointCloud<PointCloudT> > PointCloudPtr;
	typedef boost::shared_ptr< PointCloud<PointCloudT> const> PointCloudConstPtr;

    PointCloud(){};

    virtual ~PointCloud(){};

    /**
     * @brief Stub for returning a point cloud iterator.
     *
     * This function binds this abstract point cloud container with a concrete
     * iterator implementation.
     *
     * @note Please make sure that there is a (template) specialization for the
     * conrete PointCloud type used in the application. You probably also need
     * to implement a coutum iterator that implements the bruics_3d::IPoint3DIterator
     * interface.
     *
     * @return Generic point cloud iterator.
     */
    IPoint3DIterator::IPoint3DIteratorPtr getPointCloudIterator() {
    	return IPoint3DIterator::IPoint3DIteratorPtr(); // kind of null
    };

    boost::shared_ptr<PointCloudT> data;

};

/**
 * @brief Specialization for BRICS_3D::PointCloud3D type to return correct iterator implementation.
 */
template<>
inline IPoint3DIterator::IPoint3DIteratorPtr PointCloud<brics_3d::PointCloud3D>::getPointCloudIterator() {
	PointCloud3DIterator::PointCloud3DIteratorPtr it(new brics_3d::PointCloud3DIterator());
	it->insert(data);
	return it;
}

} // namespace brics_3d::rsg

} // namespace brics_3d
#endif

/* EOF */

