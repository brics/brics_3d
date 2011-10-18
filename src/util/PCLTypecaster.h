/*
 * PCLTypeCaster.h
 *
 *  Created on: Oct 7, 2011
 *      Author: Pinaki Sunil Banerjee
 */

#ifndef PCLTYPECASTER_H_
#define PCLTYPECASTER_H_

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include "core/ColoredPointCloud3D.h"
#include "core/ColoredPoint3D.h"
#include "core/ColorSpaceConvertor.h"

#include <stdio.h>
namespace BRICS_3D {

class PCLTypecaster {
public:
	PCLTypecaster();
	virtual ~PCLTypecaster();


	/**
	 * Converts from XYZ-cloud format of BRICS_3D to XYZ-cloud format of PCL
	 * @param pclCloudPtr		point cloud data in PCL format
	 * @param pointCloud3DPtr	point cloud datda in BRICS_3D format
	 */
	 void convertToPCLDataType(pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloudPtr,
			BRICS_3D::PointCloud3D* pointCloud3DPtr );



	/**
	 * Converts from XYZ-cloud format of PCL to XYZ-cloud format of BRICS_3D
	 * @param pclCloudPtr		point cloud data in PCL format
	 * @param pointCloud3DPtr	point cloud datda in BRICS_3D format
	 */
	void convertToBRICS3DDataType(pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloudPtr,
			BRICS_3D::PointCloud3D* pointCloud3DPtr );




	/**
	 * Converts from XYZRGB-cloud format of PCL to XYZ-cloud format of BRICS_3D
	 * @param pclCloudPtr		point cloud data in PCL format
	 * @param pointCloud3DPtr	point cloud datda in BRICS_3D format
	 */
	void convertToBRICS3DDataType(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclCloudPtr,
			BRICS_3D::PointCloud3D* pointCloud3DPtr );



	/**
	 * Converts from XYZRGB-cloud format of BRICS_3D to XYZRGB-cloud format of PCL
	 * @param pclCloudPtr		point cloud data in PCL format
	 * @param pointCloud3DPtr	point cloud data in BRICS_3D format
	 */
	void convertToPCLDataType(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclCloudPtr,
			BRICS_3D::ColoredPointCloud3D* pointCloud3DPtr );


	/**
	 * Converts from XYZRGB-cloud format of BRICS_3D to XYZ format of PCL
	 * @param pclCloudPtr		point cloud data in PCL format
	 * @param pointCloud3DPtr	point cloud data in BRICS_3D format
	 */
	void convertToPCLDataType(pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloudPtr,
			BRICS_3D::ColoredPointCloud3D* pointCloud3DPtr );


	/**
	 * Converts from XYZRGB-cloud format of PCL to XYZRGB-cloud format of BRICS_3D
	 * @param pclCloudPtr		point cloud data in PCL format
	 * @param pointCloud3DPtr	point cloud datda in BRICS_3D format
	 */
	void convertToBRICS3DDataType(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclCloudPtr,
			BRICS_3D::ColoredPointCloud3D* pointCloud3DPtr );

};

}

#endif /* PCLTYPECASTER_H_ */
