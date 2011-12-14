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

#ifndef ISEGMENTATION_H_
#define ISEGMENTATION_H_

#include "core/PointCloud3D.h"
#include "core/ColoredPointCloud3D.h"
#include <iostream>

namespace BRICS_3D{

class ISegmentation{

protected:


	/** @brief The input point-cloud to be processed*/
	PointCloud3D* inputPointCloud;


	/** @brief The input point-cloud to be processed*/
	ColoredPointCloud3D* inputPointCloudColored;

	/**@brief Indicates if the input is colored or not*/
	bool isColoredInput;

public:

	//	 virtual ISegmentation();


	/** @brief Set the pointcloud to be used
	 *  @param pointer to the input pointcloud
	 */
	inline void
	setPointCloud (PointCloud3D* inputPointCloud)
	{
		this->inputPointCloud = inputPointCloud;
		isColoredInput = false;
	}


	/** @brief Set the pointcloud to be used
	 *  @param pointer to the input pointcloud
	 */
	inline void
	setPointCloud (ColoredPointCloud3D* inputPointCloudColored)
	{
		this->inputPointCloudColored = inputPointCloudColored;
		isColoredInput = true;
	}


	/** @brief Get the pointcloud being used
	 *  @param pointer to the pointcloud being used
	 */
	inline void
	getPointCloud (PointCloud3D* pointCloud)
	{
		pointCloud = this->inputPointCloud;
	}


	/** @brief Get the pointcloud being used
	 *  @param pointer to the pointcloud being used
	 */
	inline void
	getPointCloud (ColoredPointCloud3D* pointCloudColored)
	{
		pointCloudColored = this->inputPointCloudColored;
	}


	/**
	 * Abstract method to perform segmentation
	 */
	virtual int segment() = 0;

	//	 virtual ~ISegmentation();
};


}



#endif /* ISEGMENTATION_H_ */
