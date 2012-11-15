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

#ifndef BRICS_3D_IPOINT3DITERATOR_H_
#define BRICS_3D_IPOINT3DITERATOR_H_

#include <boost/shared_ptr.hpp>
#include "Point3D.h"

namespace brics_3d {

//template <typename ScalarT, typename PointT> //TODO templated version?

/**
 * @brief Abstract interface to iterate over a set of points.
 *
 * Preferrably use ther getter methods for x,y and z as the iterator implementation will put the data in the right context.
 * E.g. it will apply some rigid transform  before returning the data.
 *
 * Typically you will have some code snipped like this:
 *
 *  @code
 *	IPoint3DIterator* it = new SomeImplementation();
 *	for (it->begin(); !it->end(); it->next()) {
 *		it->getX();
 *	 	it->getY();
 *		it->getZ();
 *	}
 *	delete it;
 * @endcode
 */
class IPoint3DIterator {
public:

	typedef boost::shared_ptr<IPoint3DIterator> IPoint3DIteratorPtr;
	typedef boost::shared_ptr<IPoint3DIterator const> IPoint3DIteratorConstPtr;

	/**
	 * @brief Default constructor.
	 */
	IPoint3DIterator(){};

	/**
	 * @brief Default destructor.
	 */
	virtual ~IPoint3DIterator(){};

	/**
	 * @brief Return a hint what kint of type the underlying point cloud is.
	 * This is mainly meant for debugging purposes.
	 * @return A string that identifies the type. E.g. brics_3d::PointCloud3D.
	 */
	virtual std::string getPointCloudTypeName() = 0;

	/**
	 * @brief Set the iterator to the begining of the point set.
	 */
	virtual void begin() = 0;

	/**
	 * @brief Advance to the next point.
	 */
	virtual void next() = 0;

	/**
	 * @brief Check whether the end has been reached or not.
	 * The "end" itself is not a valid element it self.
	 * @return True if end has been reached.
	 */
	virtual bool end() = 0;

	/**
	 * @brief Get the x coordiante of the point.
	 * This value might be transformed/modified depending the the iterator implementation.
	 */
	virtual Coordinate getX() = 0; // (possibly) transformed

	/**
	 * @brief Get the y coordiante of the point.
	 * This value might be transformed/modified depending the the iterator implementation.
	 */
	virtual Coordinate getY() = 0; // (possibly) transformed

	/**
	 * @brief Get the z coordiante of the point.
	 * This value might be transformed/modified depending the the iterator implementation.
	 */
	virtual Coordinate getZ() = 0; // (possibly) transformed

	/**
	 * @brief Get the unmodified data (including x,y,z) from the ith point in the iteration.
	 * @return Pointer to the original point data. This will give acces to potential data in the decoration layers. cf. brics_3d::Point3DDecorator
	 */
	virtual Point3D* getRawData() = 0; //not transformed, but might have additional data like color, etc.

};

}

#endif /* BRICS_3D_IPOINT3DITERATOR_H_ */

/* EOF */
