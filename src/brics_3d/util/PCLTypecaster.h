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

#ifndef BRICS_3D_PCLTYPECASTER_H_
#define BRICS_3D_PCLTYPECASTER_H_

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include "brics_3d/core/ColoredPoint3D.h"
#include "brics_3d/core/ColorSpaceConvertor.h"
#include "brics_3d/core/PointCloud3D.h"
#include <cmath>

#include <stdio.h>
namespace brics_3d {

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
			brics_3d::PointCloud3D* pointCloud3DPtr ) {
			pclCloudPtr->width = pointCloud3DPtr->getSize();
			pclCloudPtr->height = 1;
			pclCloudPtr->points.resize( pclCloudPtr->width * pclCloudPtr->height );

			for (unsigned int i =0 ; i<pointCloud3DPtr->getSize() ; i++){
				pclCloudPtr->points[i].x = (*pointCloud3DPtr->getPointCloud())[i].getX();
				pclCloudPtr->points[i].y = (*pointCloud3DPtr->getPointCloud())[i].getY();
				pclCloudPtr->points[i].z = (*pointCloud3DPtr->getPointCloud())[i].getZ();
			}
		}

	/**
	 * Converts from XYZ-cloud format of PCL to XYZ-cloud format of BRICS_3D
	 * @param pclCloudPtr		point cloud data in PCL format
	 * @param pointCloud3DPtr	point cloud datda in BRICS_3D format
	 */
	inline void convertToBRICS3DDataType(pcl::PointCloud<pcl::PointXYZ>::ConstPtr pclCloudPtr,
			brics_3d::PointCloud3D* pointCloud3DPtr ){
		pointCloud3DPtr->getPointCloud()->resize(pclCloudPtr->size());

		for (unsigned int i =0 ; i < pclCloudPtr->size()  ; i++){
			if(!std::isnan(pclCloudPtr->points[i].x) && !std::isinf(pclCloudPtr->points[i].x) &&
					!std::isnan(pclCloudPtr->points[i].y) && !std::isinf(pclCloudPtr->points[i].y) &&
					!std::isnan(pclCloudPtr->points[i].z) && !std::isinf(pclCloudPtr->points[i].z) ) {

			(*pointCloud3DPtr->getPointCloud())[i].setX(pclCloudPtr->points[i].x);
			(*pointCloud3DPtr->getPointCloud())[i].setY(pclCloudPtr->points[i].y);
			(*pointCloud3DPtr->getPointCloud())[i].setZ(pclCloudPtr->points[i].z);
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
			brics_3d::PointCloud3D* pointCloud3DPtr, bool copyColorData=true ){

		uint8_t r, g, b;
		brics_3d::ColorSpaceConvertor colorSpaceConvertor;
		uint32_t rgbVal;
		unsigned char red, green, blue;
		float rgbVal24Bit;
//		pointCloud3DPtr->getPointCloud()->resize(pclCloudPtr->size());

		for (unsigned int i =0 ; i < pclCloudPtr->size()  ; i++){
			if(!std::isnan(pclCloudPtr->points[i].x) && !std::isinf(pclCloudPtr->points[i].x) &&
					!std::isnan(pclCloudPtr->points[i].y) && !std::isinf(pclCloudPtr->points[i].y) &&
					!std::isnan(pclCloudPtr->points[i].z) && !std::isinf(pclCloudPtr->points[i].z) ) {

				Point3D* tmpPoint =  new Point3D(pclCloudPtr->points[i].x, pclCloudPtr->points[i].y, pclCloudPtr->points[i].z);

				if(copyColorData) {
					rgbVal24Bit = pclCloudPtr->points[i].rgb;
					rgbVal= *reinterpret_cast<int*>(&rgbVal24Bit);
					colorSpaceConvertor.rgb24bitToRGB(rgbVal, &r, &g, &b);
					red= *reinterpret_cast<unsigned char*>(&r);
					green= *reinterpret_cast<unsigned char*>(&g);
					blue= *reinterpret_cast<unsigned char*>(&b);

					ColoredPoint3D* tmpColoredPoint = new ColoredPoint3D(tmpPoint, red, green, blue);
					pointCloud3DPtr->addPointPtr(tmpColoredPoint);
				} else {
					pointCloud3DPtr->addPointPtr(tmpPoint);
				}

			}
		}

	}


	/**
	 * Converts from XYZRGB-cloud format of BRICS_3D to XYZRGB-cloud format of PCL
	 * @param pclCloudPtr		point cloud data in PCL format
	 * @param pointCloud3DPtr	point cloud data in BRICS_3D format
	 */
	inline void convertToPCLDataType(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclCloudPtr,
				brics_3d::PointCloud3D* pointCloud3DPtr ) {

		brics_3d::ColorSpaceConvertor colorSpaceConvertor;

		pclCloudPtr->width = pointCloud3DPtr->getSize();
		pclCloudPtr->height = 1;
		pclCloudPtr->points.resize( pclCloudPtr->width * pclCloudPtr->height );

		float rgbVal24bit=0;

		for (unsigned int i =0 ; i<pointCloud3DPtr->getSize() ; i++){
			pclCloudPtr->points[i].x = (*pointCloud3DPtr->getPointCloud())[i].getX();
			pclCloudPtr->points[i].y = (*pointCloud3DPtr->getPointCloud())[i].getY();
			pclCloudPtr->points[i].z = (*pointCloud3DPtr->getPointCloud())[i].getZ();

			if( (*pointCloud3DPtr->getPointCloud())[i].asColoredPoint3D() == 0) {
				/*this point does not contain color information so provide the default */

			} else {
				colorSpaceConvertor.rgbToRGB24Bit(&rgbVal24bit,
						(*pointCloud3DPtr->getPointCloud())[i].asColoredPoint3D()->getR(),
						(*pointCloud3DPtr->getPointCloud())[i].asColoredPoint3D()->getG(),
						(*pointCloud3DPtr->getPointCloud())[i].asColoredPoint3D()->getB());

			}

			pclCloudPtr->points[i].rgb = rgbVal24bit;

		}
	}
};

}

#endif /* BRICS_3D_PCLTYPECASTER_H_ */
