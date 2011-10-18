/*
 * PCLTypeCaster.cpp
 *
 *  Created on: Oct 7, 2011
 *      Author: Pinaki Sunil Banerjee
 */


#include "PCLTypecaster.h"

namespace BRICS_3D {

PCLTypecaster::PCLTypecaster() {}

PCLTypecaster::~PCLTypecaster() {}


void PCLTypecaster::convertToPCLDataType(pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloudPtr,
		BRICS_3D::PointCloud3D* pointCloud3DPtr ){
//printf("[PCLTypeCaster]iNPUT cloud Size: %d \n", pointCloud3DPtr->getSize());
	pclCloudPtr->width = pointCloud3DPtr->getSize();
	pclCloudPtr->height = 1;
	pclCloudPtr->points.resize( pclCloudPtr->width * pclCloudPtr->height );

	for (unsigned int i =0 ; i<pointCloud3DPtr->getSize() ; i++){
		pclCloudPtr->points[i].x = pointCloud3DPtr->getPointCloud()->data()[i].getX();
		pclCloudPtr->points[i].y = pointCloud3DPtr->getPointCloud()->data()[i].getY();
		pclCloudPtr->points[i].z = pointCloud3DPtr->getPointCloud()->data()[i].getZ();
	}
//printf("[PCLTypeCaster]cONVERTED cloud Size: %d \n", pclCloudPtr->size());
}





void PCLTypecaster::convertToBRICS3DDataType(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclCloudPtr,
		BRICS_3D::PointCloud3D* pointCloud3DPtr ){
//	//printf("[PCLTypeCaster]Input cloud Size: %d \n", pclCloudPtr->size());
	for (unsigned int i =0 ; i < pclCloudPtr->size()  ; i++){
			pointCloud3DPtr->addPoint(ColoredPoint3D(new Point3D(pclCloudPtr->points[i].x,
							pclCloudPtr->points[i].y, pclCloudPtr->points[i].z)));
		}
//	//printf("[PCLTypeCaster]Converted cloud Size: %d \n", pointCloud3DPtr->getSize());
}



void PCLTypecaster::convertToBRICS3DDataType(pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloudPtr,
		BRICS_3D::PointCloud3D* pointCloud3DPtr ){

//	pointCloud3DPtr->getPointCloud()->resize(pclCloudPtr->size());

	for (unsigned int i =0 ; i < pclCloudPtr->size()  ; i++){
		pointCloud3DPtr->addPoint(ColoredPoint3D(new Point3D(pclCloudPtr->points[i].x,
						pclCloudPtr->points[i].y, pclCloudPtr->points[i].z)));
		}
}



void PCLTypecaster::convertToPCLDataType(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclCloudPtr,
		BRICS_3D::ColoredPointCloud3D* pointCloud3DPtr ){

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


void PCLTypecaster::convertToPCLDataType(pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloudPtr,
		BRICS_3D::ColoredPointCloud3D* pointCloud3DPtr ){

	pclCloudPtr->width = pointCloud3DPtr->getSize();
	pclCloudPtr->height = 1;
	pclCloudPtr->points.resize( pclCloudPtr->width * pclCloudPtr->height );

	float rgbVal24bit=0;

	for (unsigned int i =0 ; i<pointCloud3DPtr->getSize() ; i++){
		pclCloudPtr->points[i].x = pointCloud3DPtr->getPointCloud()->data()[i].getX();
		pclCloudPtr->points[i].y = pointCloud3DPtr->getPointCloud()->data()[i].getY();
		pclCloudPtr->points[i].z = pointCloud3DPtr->getPointCloud()->data()[i].getZ();
	}
}


void PCLTypecaster::convertToBRICS3DDataType(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclCloudPtr,
		BRICS_3D::ColoredPointCloud3D* pointCloud3DPtr ){

	uint8_t r, g, b;
	BRICS_3D::ColorSpaceConvertor colorSpaceConvertor;
	uint32_t rgbVal;
	unsigned char red, green, blue;
	////printf("\n\n\tCHECKPOINT!!!\n");
	for (unsigned int i =0 ; i < pclCloudPtr->size()  ; i++){
		rgbVal= *reinterpret_cast<int*>(&pclCloudPtr->points[i].rgb);
		colorSpaceConvertor.rgb24bitToRGB(rgbVal, &r, &g, &b);
		red= *reinterpret_cast<unsigned char*>(&r);
		green= *reinterpret_cast<unsigned char*>(&g);
		blue= *reinterpret_cast<unsigned char*>(&b);
 		pointCloud3DPtr->addPoint(new ColoredPoint3D(new Point3D(pclCloudPtr->points[i].x,
				pclCloudPtr->points[i].y, pclCloudPtr->points[i].z), red, green, blue));
	}

}


}
