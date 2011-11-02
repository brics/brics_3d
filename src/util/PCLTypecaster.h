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

#ifndef PCLTYPECASTER_H_
#define PCLTYPECASTER_H_

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include "core/ColoredPointCloud3D.h"
#include "core/ColoredPoint3D.h"
#include "core/ColorSpaceConvertor.h"
#include <cmath>

#include <stdio.h>
namespace BRICS_3D {

class PCLTypecaster {
public:
	PCLTypecaster(){};
	virtual ~PCLTypecaster(){};


	/**
	 * Converts from XYZ-cloud format of BRICS_3D to XYZ-cloud format of PCL
	 * @param pclCloudPtr		point cloud data in PCL format
	 * @param pointCloud3DPtr	point cloud datda in BRICS_3D format
	 */
	inline void convertToPCLDataType(pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloudPtr,
			BRICS_3D::PointCloud3D* pointCloud3DPtr ) {
			pclCloudPtr->width = pointCloud3DPtr->getSize();
			pclCloudPtr->height = 1;
			pclCloudPtr->points.resize( pclCloudPtr->width * pclCloudPtr->height );

			for (unsigned int i =0 ; i<pointCloud3DPtr->getSize() ; i++){
				pclCloudPtr->points[i].x = pointCloud3DPtr->getPointCloud()->data()[i].getX();
				pclCloudPtr->points[i].y = pointCloud3DPtr->getPointCloud()->data()[i].getY();
				pclCloudPtr->points[i].z = pointCloud3DPtr->getPointCloud()->data()[i].getZ();
			}
		}

	/**
	 * Converts from XYZ-cloud format of PCL to XYZ-cloud format of BRICS_3D
	 * @param pclCloudPtr		point cloud data in PCL format
	 * @param pointCloud3DPtr	point cloud datda in BRICS_3D format
	 */
	inline void convertToBRICS3DDataType(pcl::PointCloud<pcl::PointXYZ>::ConstPtr pclCloudPtr,
			BRICS_3D::PointCloud3D* pointCloud3DPtr ){
		pointCloud3DPtr->getPointCloud()->resize(pclCloudPtr->size());

		for (unsigned int i =0 ; i < pclCloudPtr->size()  ; i++){
			if(!std::isnan(pclCloudPtr->points[i].x) && !std::isinf(pclCloudPtr->points[i].x) &&
					!std::isnan(pclCloudPtr->points[i].y) && !std::isinf(pclCloudPtr->points[i].y) &&
					!std::isnan(pclCloudPtr->points[i].z) && !std::isinf(pclCloudPtr->points[i].z) ) {

			pointCloud3DPtr->getPointCloud()->data()[i].setX(pclCloudPtr->points[i].x);
			pointCloud3DPtr->getPointCloud()->data()[i].setY(pclCloudPtr->points[i].y);
			pointCloud3DPtr->getPointCloud()->data()[i].setZ(pclCloudPtr->points[i].z);
			//		pointCloud3DPtr->addPoint(new Point3D(pclCloudPtr->points[i].x,
			//							pclCloudPtr->points[i].y, pclCloudPtr->points[i].z));

			}
		}
	}



	/**
	 * Converts from XYZRGB-cloud format of PCL to XYZ-cloud format of BRICS_3D
	 * @param pclCloudPtr		point cloud data in PCL format
	 * @param pointCloud3DPtr	point cloud datda in BRICS_3D format
	 */
	inline void convertToBRICS3DDataType(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr pclCloudPtr,
			BRICS_3D::PointCloud3D* pointCloud3DPtr ){


		pointCloud3DPtr->getPointCloud()->resize(pclCloudPtr->size());

		for (unsigned int i =0 ; i < pclCloudPtr->size()  ; i++){
			if(!std::isnan(pclCloudPtr->points[i].x) && !std::isinf(pclCloudPtr->points[i].x) &&
					!std::isnan(pclCloudPtr->points[i].y) && !std::isinf(pclCloudPtr->points[i].y) &&
					!std::isnan(pclCloudPtr->points[i].z) && !std::isinf(pclCloudPtr->points[i].z) ) {

			pointCloud3DPtr->getPointCloud()->data()[i].setX(pclCloudPtr->points[i].x);
			pointCloud3DPtr->getPointCloud()->data()[i].setY(pclCloudPtr->points[i].y);
			pointCloud3DPtr->getPointCloud()->data()[i].setZ(pclCloudPtr->points[i].z);

			//		pointCloud3DPtr->addPoint(new Point3D(pclCloudPtr->points[i].x,
			//							pclCloudPtr->points[i].y, pclCloudPtr->points[i].z));

			}
		}
	}


	/**
	 * Converts from XYZRGB-cloud format of BRICS_3D to XYZRGB-cloud format of PCL
	 * @param pclCloudPtr		point cloud data in PCL format
	 * @param pointCloud3DPtr	point cloud data in BRICS_3D format
	 */
	inline void convertToPCLDataType(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclCloudPtr,
				BRICS_3D::ColoredPointCloud3D* pointCloud3DPtr ) {

		BRICS_3D::ColorSpaceConvertor colorSpaceConvertor;

		pclCloudPtr->width = pointCloud3DPtr->getSize();
		pclCloudPtr->height = 1;
		pclCloudPtr->points.resize( pclCloudPtr->width * pclCloudPtr->height );

		float rgbVal24bit=0;

		for (unsigned int i =0 ; i<pointCloud3DPtr->getSize() ; i++){
			pclCloudPtr->points[i].x = pointCloud3DPtr->getPointCloud()->data()[i].getX();
			pclCloudPtr->points[i].y = pointCloud3DPtr->getPointCloud()->data()[i].getY();
			pclCloudPtr->points[i].z = pointCloud3DPtr->getPointCloud()->data()[i].getZ();
			colorSpaceConvertor.rgbToRGB24Bit(&rgbVal24bit,
					pointCloud3DPtr->getPointCloud()->data()[i].getR(),
					pointCloud3DPtr->getPointCloud()->data()[i].getG(),
					pointCloud3DPtr->getPointCloud()->data()[i].getB());
			pclCloudPtr->points[i].rgb = rgbVal24bit;
		}
	}


	/**
	 * Converts from XYZRGB-cloud format of BRICS_3D to XYZ format of PCL
	 * @param pclCloudPtr		point cloud data in PCL format
	 * @param pointCloud3DPtr	point cloud data in BRICS_3D format
	 */
	inline void convertToPCLDataType(pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloudPtr,
			BRICS_3D::ColoredPointCloud3D* pointCloud3DPtr ){

		pclCloudPtr->width = pointCloud3DPtr->getSize();
		pclCloudPtr->height = 1;
		pclCloudPtr->points.resize( pclCloudPtr->width * pclCloudPtr->height );

		for (unsigned int i =0 ; i<pointCloud3DPtr->getSize() ; i++){
			pclCloudPtr->points[i].x = pointCloud3DPtr->getPointCloud()->data()[i].getX();
			pclCloudPtr->points[i].y = pointCloud3DPtr->getPointCloud()->data()[i].getY();
			pclCloudPtr->points[i].z = pointCloud3DPtr->getPointCloud()->data()[i].getZ();
		}
	}


	/**
	 * Converts from XYZRGB-cloud format of PCL to XYZRGB-cloud format of BRICS_3D
	 * @param pclCloudPtr		point cloud data in PCL format
	 * @param pointCloud3DPtr	point cloud datda in BRICS_3D format
	 */
	inline void convertToBRICS3DDataType(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr pclCloudPtr,
			BRICS_3D::ColoredPointCloud3D* pointCloud3DPtr ){

		uint8_t r, g, b;
		BRICS_3D::ColorSpaceConvertor colorSpaceConvertor;
		uint32_t rgbVal;
		unsigned char red, green, blue;
		float rgbVal24Bit;
		//	printf("Ptr location: %d \n", pclCloud);
		printf("Input Cloud Size: %d \n", pclCloudPtr->size());

		//FIXME Results into a memory leak
		//	pointCloud3DPtr->getPointCloud()->resize(pclCloudPtr->size());
		//
		for (unsigned int i =0 ; i < pclCloudPtr->size()  ; i++){

			if(!std::isnan(pclCloudPtr->points[i].x) && !std::isinf(pclCloudPtr->points[i].x) &&
					!std::isnan(pclCloudPtr->points[i].y) && !std::isinf(pclCloudPtr->points[i].y) &&
					!std::isnan(pclCloudPtr->points[i].z) && !std::isinf(pclCloudPtr->points[i].z) ) {
				rgbVal24Bit = pclCloudPtr->points[i].rgb;
				rgbVal= *reinterpret_cast<int*>(&rgbVal24Bit);
				colorSpaceConvertor.rgb24bitToRGB(rgbVal, &r, &g, &b);
				red= *reinterpret_cast<unsigned char*>(&r);
				green= *reinterpret_cast<unsigned char*>(&g);
				blue= *reinterpret_cast<unsigned char*>(&b);
				Point3D* tmpPoint =  new Point3D(pclCloudPtr->points[i].x, pclCloudPtr->points[i].y, pclCloudPtr->points[i].z);
				ColoredPoint3D* tmpColoredPoint = new ColoredPoint3D(tmpPoint, red, green, blue);
				pointCloud3DPtr->addPoint(tmpColoredPoint);
				delete tmpPoint;
				delete tmpColoredPoint;
				//		pointCloud3DPtr->getPointCloud()->data()[i].setX(pclCloudPtr->points[i].x);
				//		pointCloud3DPtr->getPointCloud()->data()[i].setY(pclCloudPtr->points[i].y);
				//		pointCloud3DPtr->getPointCloud()->data()[i].setZ(pclCloudPtr->points[i].z);
				//		pointCloud3DPtr->getPointCloud()->data()[i].setR(red);
				//		pointCloud3DPtr->getPointCloud()->data()[i].setG(blue);
				//		pointCloud3DPtr->getPointCloud()->data()[i].setB(green);
			}
		}
	}
};

}

#endif /* PCLTYPECASTER_H_ */
