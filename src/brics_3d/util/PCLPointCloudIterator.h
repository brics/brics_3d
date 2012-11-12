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

#ifndef BRICS_3D_PCLPOINTCLOUDITERATOR_H_
#define BRICS_3D_PCLPOINTCLOUDITERATOR_H_

#include "brics_3d/core/IPoint3DIterator.h"
#include "pcl/point_cloud.h"

namespace brics_3d {

/**
 * @brief Iterator implementation for PCL point clouds.
 */
template <typename PointT>
class PCLPointCloudIterator : public IPoint3DIterator {
public:

	/**
	 * @brief Default constructor.
	 */
	PCLPointCloudIterator(){
		currentRawPoint = 0;
		begin();
	};

	/**
	 * @brief Default destructor.
	 */
	virtual ~PCLPointCloudIterator(){
		if (currentRawPoint) {
			delete currentRawPoint;
			currentRawPoint = 0;
		}
	};

	void begin(){
		index = 0;
		pointCloudsIterator = pointCloudsWithTransforms.begin();


		while(!end() && (pointCloudsIterator->first->points.size() <= 0)) { //skip empty clouds
			pointCloudsIterator++;
			LOG(WARNING) << "PCL Iterator contains empty point clouds.";
		}

		if ( !end() ) {// end could be reached meanwhile so we have to check again
			/* cache a transformed copy */
//			LOG(DEBUG) << "PCL cloud has " << pointCloudsIterator->first->points.size() << " points.";
			currentTransformedPoint.x = pointCloudsIterator->first->points[index].x;
			currentTransformedPoint.y = pointCloudsIterator->first->points[index].y;
			currentTransformedPoint.z = pointCloudsIterator->first->points[index].z;
			homogeneousTransformation(currentTransformedPoint, pointCloudsIterator->second);
		} else {
			pointCloudsIterator = pointCloudsWithTransforms.end(); // empty iterator
		}
	};

	void next(){
		++index;

		if ( !end() ) {
			if(index >= pointCloudsIterator->first->points.size()) { //wrap over - advance to next point cloud
				index = 0;
				pointCloudsIterator++;
				while(!end() && (pointCloudsIterator->first->points.size() <= 0)) { //skip further empty clouds
					pointCloudsIterator++;
					LOG(WARNING) << "PCL Iterator contains empty point clouds.";
				}
			}

			if ( !end() ) { // end could be reached meanwhile so we have to check again
				if (false && pointCloudsIterator->second->isIdentity()) { // "lazyness" case //TODO: isIdentity is really slow!
					/* cache the real data */
					currentTransformedPoint.x = pointCloudsIterator->first->points[index].x;
					currentTransformedPoint.y = pointCloudsIterator->first->points[index].y;
					currentTransformedPoint.z = pointCloudsIterator->first->points[index].z;

				} else {

					/* cache the a transformed copy */
					currentTransformedPoint.x = pointCloudsIterator->first->points[index].x;
					currentTransformedPoint.y = pointCloudsIterator->first->points[index].y;
					currentTransformedPoint.z = pointCloudsIterator->first->points[index].z;
					homogeneousTransformation(currentTransformedPoint, pointCloudsIterator->second);
				}
			}
		} else {
			/* no further iterations, we are at the end */
		}
	};

	bool end(){
		if (pointCloudsIterator ==  pointCloudsWithTransforms.end()) {
				return true;
		}
		return false;
	}

	Coordinate getX() { // (possibly) transformed
		return currentTransformedPoint.x;
	};

	Coordinate getY() { // (possibly) transformed
		return currentTransformedPoint.y;
	};

	Coordinate getZ() { // (possibly) transformed
		return currentTransformedPoint.z;
	};

	Point3D* getRawData() { //not transformed, but might have additional data like color, etc.
		if (currentRawPoint) { //delete old copy
			delete currentRawPoint;
			currentRawPoint = 0;
		}
		currentRawPoint = new Point3D(pointCloudsIterator->first->points[index].x,
				pointCloudsIterator->first->points[index].y,
				pointCloudsIterator->first->points[index].z);

		return currentRawPoint;
	}

	/**
	 * @brief Insert a new point cloud tht will be acessable via the iterator
	 * @param pointCloud The point cloud.
	 * @param associatedTransform An associated transform. It will be automatically applied to getX(), getY() and getZ()
	 */
	void insert(const typename pcl::PointCloud<PointT>::ConstPtr pointCloud, IHomogeneousMatrix44::IHomogeneousMatrix44Ptr associatedTransform) {
		pointCloudsWithTransforms.insert(std::make_pair(pointCloud, associatedTransform));
	}

	/**
	 * @brief Apply a brics_3d transform to a PCL point type.
	 * @param point Templated PCL point type.
	 * @param transformation Transform to be applied.
	 */
	static inline void homogeneousTransformation(PointT& point, IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transformation) {
		const double *homogenousMatrix;
		double xTemp;
		double yTemp;
		double zTemp;

		homogenousMatrix = transformation->getRawData();

		/*
		 * layout:
		 * 0 4 8  12
		 * 1 5 9  13
		 * 2 6 10 14
		 * 3 7 11 15
		 */

		/* rotate */;
		xTemp = point.x * homogenousMatrix[0] + point.y * homogenousMatrix[4] + point.z * homogenousMatrix[8];
		yTemp = point.x * homogenousMatrix[1] + point.y * homogenousMatrix[5] + point.z * homogenousMatrix[9];
		zTemp = point.x * homogenousMatrix[2] + point.y * homogenousMatrix[6] + point.z * homogenousMatrix[10];

		/* translate */
		point.x = xTemp + homogenousMatrix[12];
		point.y = yTemp + homogenousMatrix[13];
		point.z = zTemp + homogenousMatrix[14];
//		LOG(DEBUG) << "Transformed point = (" << point.x << ", " << point.y << ", " << point.y << ")";
	}

protected:

	/**
	 * The stored pointers to the point clouds with associated transforms.
	 * Destruction of the iterator will not delete the pointers.
	 */
	std::map<const typename pcl::PointCloud<PointT>::ConstPtr, IHomogeneousMatrix44::IHomogeneousMatrix44Ptr> pointCloudsWithTransforms;

	/// Internal outer iteration handle.
	typename std::map<const typename pcl::PointCloud<PointT>::ConstPtr, IHomogeneousMatrix44::IHomogeneousMatrix44Ptr>::iterator pointCloudsIterator;

	/// Internal inner iteration handle.
	unsigned int index;

	/// The cached data.
	PointT currentTransformedPoint;

	/// Converted type for getRawData method
	brics_3d::Point3D* currentRawPoint;

};

}

#endif /* BRICS_3D_PCLPOINTCLOUDITERATOR_H_ */

/* EOF */
