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
	pclCloudPtr->width = pointCloud3DPtr->getSize();
	pclCloudPtr->height = 1;
	pclCloudPtr->points.resize( pclCloudPtr->width * pclCloudPtr->height );

	for (unsigned int i =0 ; i<pointCloud3DPtr->getSize() ; i++){
		pclCloudPtr->points[i].x = pointCloud3DPtr->getPointCloud()->data()[i].getX();
		pclCloudPtr->points[i].y = pointCloud3DPtr->getPointCloud()->data()[i].getY();
		pclCloudPtr->points[i].z = pointCloud3DPtr->getPointCloud()->data()[i].getZ();
	}
}





void PCLTypecaster::convertToBRICS3DDataType(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr pclCloudPtr,
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



void PCLTypecaster::convertToBRICS3DDataType(pcl::PointCloud<pcl::PointXYZ>::ConstPtr pclCloudPtr,
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

	for (unsigned int i =0 ; i<pointCloud3DPtr->getSize() ; i++){
		pclCloudPtr->points[i].x = pointCloud3DPtr->getPointCloud()->data()[i].getX();
		pclCloudPtr->points[i].y = pointCloud3DPtr->getPointCloud()->data()[i].getY();
		pclCloudPtr->points[i].z = pointCloud3DPtr->getPointCloud()->data()[i].getZ();
	}
}


void PCLTypecaster::convertToBRICS3DDataType(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr pclCloudPtr,
		BRICS_3D::ColoredPointCloud3D* pointCloud3DPtr ){

	uint8_t r, g, b;
	BRICS_3D::ColorSpaceConvertor colorSpaceConvertor;
	uint32_t rgbVal;
	unsigned char red, green, blue;
	float rgbVal24Bit;
	//	printf("Ptr location: %d \n", pclCloud);
	//	printf("Input Cloud Size: %d \n", pclCloudPtr->size());

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

	//printf("Output Cloud Size: %d", pointCloud3DPtr->getSize());

}

}
