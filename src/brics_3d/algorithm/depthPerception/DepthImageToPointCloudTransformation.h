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

#ifndef BRICS_3D_DEPTHIMAGETOPOINTCLOUDTRANSFORMATION_H_
#define BRICS_3D_DEPTHIMAGETOPOINTCLOUDTRANSFORMATION_H_

#include "core/PointCloud3D.h"
#include <cv.h>
#include <highgui.h>

/// Maximum value for depth image as it is assumed to be an (unsigned) 8bit grayscale image
#define MAX_DEPTHIMAGE_VALUE 255

/**
 * @namespace brics_3d
 */
namespace brics_3d {

/**
 * @brief Transforms a depth image into a point cloud.
 *
 * @ingroup depth_perception
 *
 *  Point cloud frame:
 *
 *  z&nbsp;&nbsp;&nbsp;&nbsp;y		<br>
 *  ^&nbsp;&nbsp;&nbsp;^ 			<br>
 *  |&nbsp;&nbsp;/  				<br>
 *  |&nbsp;/   						<br>
 *  |/    							<br>
 *  +--------> x
 */
/*
 *  Point cloud frame: (without HTML mess up):
 *
 *  z    y
 *  ^   ^
 *  |  /
 *  | /
 *  |/
 *  +--------> x
*/
class DepthImageToPointCloudTransformation {
public:

	/**
	 * @brief Standard constuctor
	 */
	DepthImageToPointCloudTransformation();

	/**
	 * @brief Standard destructor
	 */
	virtual ~DepthImageToPointCloudTransformation();

	/**
	 * @brief Transforms a depth image into a point cloud
	 * @param[in] depthImage Input pointer to depth image
	 * @param[out] pointCloud Output pointer to point cloud
	 * @param threshold Threshold where to cut off background pixels. Only pixels greater or equal than this threshold are taken
	 * Default is 0.0, that means all pixels are taken
	 *
	 * TODO: camera is not calibrated!!!
	 *
	 * <b>Implicit assumptions</b>:
	 *
	 * 1.) point cloud is in camera frame
	 *
	 * 2.) image is in gray scale and 8bit
	 *
	 * 3.) brighter regions appears nearer to camera
	 */
	void transformDepthImageToPointCloud(IplImage *depthImage, PointCloud3D *pointCloud, double threshold = 0.0);



};

}

#endif /* BRICS_3D_DEPTHIMAGETOPOINTCLOUDTRANSFORMATION_H_ */

/* EOF */

