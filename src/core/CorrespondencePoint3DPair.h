/**
 * @file 
 * CorrespondencePoint3DPair.h
 *
 * @date: Dec 7, 2009
 * @author: sblume
 */

#ifndef CORRESPONDENCEPOINT3DPAIR_H_
#define CORRESPONDENCEPOINT3DPAIR_H_

#include "core/Point3D.h"

namespace BRICS_3D {

/**
 * @brief Class to represent a correspondence between two Point3D entities.
 *
 */
class CorrespondencePoint3DPair {
public:

	/**
	 * @brief Standard constructor
	 */
	CorrespondencePoint3DPair();

	/**
	 * @brief Constructor that initializes the corresponding points
	 * @param firstPoint Initialize the first point
	 * @param secondPoint Initialize the second point
	 */
	CorrespondencePoint3DPair(Point3D firstPoint, Point3D secondPoint);

	/**
	 * @brief Standard destructor
	 */
	virtual ~CorrespondencePoint3DPair();

	/// First point in first set
	Point3D firstPoint;

	/// Corresponding point in second set
	Point3D secondPoint;
};

}

#endif /* CORRESPONDENCEPOINT3DPAIR_H_ */

/* EOF */
