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
#include "brics_3d/core/HomogeneousMatrix44.h"
#include "brics_3d/core/ColorSpaceConvertor.h"
#include "brics_3d/core/ColoredPoint3D.h"
#include "brics_3d/core/Logger.h"
#include "pcl/point_cloud.h"

using brics_3d::Logger;

namespace brics_3d {

/**
 * @brief Iterator implementation for PCL point clouds.
 */
template <typename PointT>
class PCLPointCloudIterator : public IPoint3DIterator {
public:

	typedef boost::shared_ptr<PCLPointCloudIterator> PCLPointCloudIteratorPtr;
	typedef boost::shared_ptr<PCLPointCloudIterator const> PCLPointCloudIteratorConstPtr;

	/**
	 * @brief Default constructor.
	 */
	PCLPointCloudIterator(){
		currentRawPoint = new Point3D();
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

	std::string getPointCloudTypeName() {
		return "pcl::PointCloud<PointT>";
	}

	void begin(){
		index = 0;
		pointCloudsIterator = pointCloudsWithTransforms.begin();
		associatedTransformIsIdentityIterator = associatedTransformIsIdentity.begin();

		while(!end() && (pointCloudsIterator->first->points.size() <= 0)) { //skip empty clouds
			pointCloudsIterator++;
			associatedTransformIsIdentityIterator++;
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
				associatedTransformIsIdentityIterator++;
				while(!end() && (pointCloudsIterator->first->points.size() <= 0)) { //skip further empty clouds
					pointCloudsIterator++;
					associatedTransformIsIdentityIterator++;
					LOG(WARNING) << "PCL Iterator contains empty point clouds.";
				}
			}

			if ( !end() ) { // end could be reached meanwhile so we have to check again

				currentTransformedPoint.x = pointCloudsIterator->first->points[index].x;
				currentTransformedPoint.y = pointCloudsIterator->first->points[index].y;
				currentTransformedPoint.z = pointCloudsIterator->first->points[index].z;
				if(*associatedTransformIsIdentityIterator == false) { // the non "lazyness" case
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
		toPoint3D(pointCloudsIterator->first->points[index], currentRawPoint);

		return currentRawPoint;
	}

	/**
	 * @brief Insert a new point cloud that will be acessable via the iterator
	 * @param pointCloud The point cloud.
	 * @param associatedTransform An associated transform. It will be automatically applied to getX(), getY() and getZ()
	 */
	void insert(const typename pcl::PointCloud<PointT>::ConstPtr pointCloud, IHomogeneousMatrix44::IHomogeneousMatrix44Ptr associatedTransform) {
		assert(pointCloud != 0);
		assert(associatedTransform != 0);
		pointCloudsWithTransforms.insert(std::make_pair(pointCloud, associatedTransform));
		associatedTransformIsIdentity.push_back(associatedTransform->isIdentity());
	}

	/**
	 * @brief Add a point cloud with an assumed identity transform.
	 * This function is for convenience.
	 * @param pointCloud The point cloud to be added.
	 */
	void insert(const typename pcl::PointCloud<PointT>::ConstPtr pointCloud) {
		brics_3d::IHomogeneousMatrix44::IHomogeneousMatrix44Ptr identityTransform(new brics_3d::HomogeneousMatrix44());
		insert(pointCloud, identityTransform);
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

	/**
	 * Shortcut if an associated transform is the Identity transform.
	 * Assumes ith entry in this vector belongs to ith entry in pointCloudsWithTransforms.
	 * By having this memory we can avoid sucessive calls of IHomogeneousMatrix44::isIdentity() -
	 * which is a rather expensive operation.
	 */
	std::vector<bool> associatedTransformIsIdentity;

	/// Iterator for associatedTransformIsIdentity.
	std::vector<bool>::iterator associatedTransformIsIdentityIterator;

	/// The cached data.
	PointT currentTransformedPoint;

	/// Converted type for getRawData method
	brics_3d::Point3D* currentRawPoint;

};

/**
 * @brief Base function for converting a generic PCL point into a BRICS_3D point.
 * @param inPoint PCL point
 * @param outPoint BRICS_3D point
 */
template<class PointT>
inline void toPoint3D(PointT& inPoint, brics_3d::Point3D*& outPoint) {
	assert(outPoint != 0);
	outPoint->setX(inPoint.x);
	outPoint->setY(inPoint.y);
	outPoint->setZ(inPoint.z);
}

/*
 * Further specializations:
 */

/// Specialization for pcl::PointXYZRGB
template<>
inline void toPoint3D(const pcl::PointXYZRGB& inPoint, brics_3d::Point3D*& outPoint){ //ref
	assert(outPoint != 0);
	outPoint->setX(inPoint.x);
	outPoint->setY(inPoint.y);
	outPoint->setZ(inPoint.z);

	if(outPoint->asColoredPoint3D() == 0) { // We need an extra decoration layer. Should be done only once.
		ColoredPoint3D* tmpColoredPoint = new ColoredPoint3D(outPoint);
		outPoint = tmpColoredPoint; // Change pointer such that the color decoration is at the outer layer
		assert(outPoint->asColoredPoint3D() != 0);
		LOG(DEBUG) << "PCLPointCloudIterator: Adding a new color decoration layer as type is PointXYZRGB.";
	}

	uint8_t r, g, b;
	uint32_t rgbVal;
	unsigned char red, green, blue;
	pcl::PointXYZRGB tmpPoint;

	tmpPoint = inPoint;
	rgbVal = *reinterpret_cast<int*>(&tmpPoint.rgb);
	r = ((rgbVal >> 16) & 0xff);
	g = ((rgbVal >> 8) & 0xff);
	b = (rgbVal & 0xff);
	red = *reinterpret_cast<unsigned char*>(&r);
	green = *reinterpret_cast<unsigned char*>(&g);
	blue = *reinterpret_cast<unsigned char*>(&b);

	outPoint->asColoredPoint3D()->setR(red);
	outPoint->asColoredPoint3D()->setG(green);
	outPoint->asColoredPoint3D()->setB(blue);
}

/// Specialization for pcl::PointXYZRGBNormal
template<>
inline void toPoint3D(const pcl::PointXYZRGBNormal& inPoint, brics_3d::Point3D*& outPoint){ //ref
	assert(outPoint != 0);
	outPoint->setX(inPoint.x);
	outPoint->setY(inPoint.y);
	outPoint->setZ(inPoint.z);

	if(outPoint->asColoredPoint3D() == 0) { // We need an extra decoration layer. Should be done only once.
		ColoredPoint3D* tmpColoredPoint = new ColoredPoint3D(outPoint);
		outPoint = tmpColoredPoint; // Change pointer such that the color decoration is at the outer layer
		assert(outPoint->asColoredPoint3D() != 0);
		LOG(DEBUG) << "PCLPointCloudIterator: Adding a new color decoration layer as type is PointXYZRGBNormal.";
	}

	uint8_t r, g, b;
	uint32_t rgbVal;
	unsigned char red, green, blue;
	pcl::PointXYZRGBNormal tmpPoint;

	tmpPoint = inPoint;
	rgbVal = *reinterpret_cast<int*>(&tmpPoint.rgb);
	r = ((rgbVal >> 16) & 0xff);
	g = ((rgbVal >> 8) & 0xff);
	b = (rgbVal & 0xff);
	red = *reinterpret_cast<unsigned char*>(&r);
	green = *reinterpret_cast<unsigned char*>(&g);
	blue = *reinterpret_cast<unsigned char*>(&b);

	outPoint->asColoredPoint3D()->setR(red);
	outPoint->asColoredPoint3D()->setG(green);
	outPoint->asColoredPoint3D()->setB(blue);
}

}

#endif /* BRICS_3D_PCLPOINTCLOUDITERATOR_H_ */

/* EOF */
