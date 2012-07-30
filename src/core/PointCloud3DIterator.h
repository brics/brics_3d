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


#ifndef POINTCLOUD3DITERATOR_H_
#define POINTCLOUD3DITERATOR_H_

#include <map>

#include "IPoint3DIterator.h"
#include "PointCloud3D.h"
#include "IHomogeneousMatrix44.h"

namespace BRICS_3D {

/**
 * @brief Point iterator impementation for BRICS_3D::PointCloud3D
 *
 * The PointCloud3DIterator can hold a list of point clouds with associated rigid transforms.
 * While iterating and invocing getX(), getY() or getZ() the points will be multiplied with this is respective transforms.
 * The raw data in the point clouds remains unmodified and could be accessed via the getRawData function.
 *
 *  @code
 *	PointCloud3D* cloud1 = new PointCloud3D();
 *	cloud1->addPoint(Point3D(1,2,3));
 *	cloud1->addPoint(Point3D(4,5,6));
 *	cloud1->addPoint(Point3D(7,8,9));

 *	PointCloud3DIterator* it = new PointCloud3DIterator();
 *	HomogeneousMatrix44 shift100(1,0,0, 0,1,0, 0,0,1, 100, 100, 100);
 *	it->insert(cloud1, &shift100);
 *
 *	for (it->begin(); !it->end(); it->next()){
 *		it->getX(); //transformed data (by "shift100")
 *		it->getY(); //transformed data (by "shift100")
 *		it->getZ(); //transformed data (by "shift100")
 *		Point3D* tmpPoint = it->getRawData();
 *		tmpPoint->getX(); // original data
 *		tmpPoint->getY(); // original data
 *		tmpPoint->getZ(); // original data
 *	}
 *	delete cloud1;
 *	delete it;
 *	@endcode
 */
class PointCloud3DIterator : public IPoint3DIterator {

public:

	/**
	 * @brief Standard constructor.
	 */
	PointCloud3DIterator();

	/**
	 * @brief Standard destructor.
	 */
	virtual ~PointCloud3DIterator();

	void begin();
	void next();
	bool end();

	virtual Coordinate getX(); // (possibly) transformed
	virtual Coordinate getY(); // (possibly) transformed
	virtual Coordinate getZ(); // (possibly) transformed
	virtual Point3D* getRawData(); //not transformed, but might have additional data like color, etc.

	/**
	 * @brief Add a point cloud with its associated transform.
	 * @param pointCloud The pont cloud to be added.
	 * @param associatedTransform Transform theat will be automatically applied to all points od that point cloud when calling getX(), getY() or getZ().
	 */
	void insert(PointCloud3D* pointCloud, IHomogeneousMatrix44* associatedTransform);

protected:

	/**
	 * The stored pointers to the point clouds with associated transforms.
	 * Destruction of the iterator will not delete the pointers.
	 */
	std::map<PointCloud3D*, IHomogeneousMatrix44*> pointCloudsWithTransforms;

	/// Internal outer iteration handle.
	std::map<PointCloud3D*, IHomogeneousMatrix44*>::iterator pointCloudsIterator;

	/// Internal inner iteration handle.
	unsigned int index;

	/// The cached data.
	Point3D* currentTransformedPoint;
};


}

#endif /* POINTCLOUD3DITERATOR_H_ */

/* EOF */
