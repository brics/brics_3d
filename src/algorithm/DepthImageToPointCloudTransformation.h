/**
 * @file
 * brics_3dpm 
 * DepthImageToPointCloudTransformation.h
 * 
 * @author: Sebastian Blumenthal
 * @date: Aug 25, 2009
 * @version: 0.1
 */

#ifndef DEPTHIMAGETOPOINTCLOUDTRANSFORMATION_H_
#define DEPTHIMAGETOPOINTCLOUDTRANSFORMATION_H_

#include "core/CartesianPointCloud.h"
#include <cv.h>
#include <highgui.h>

/// Maximum value for depth image as it is assumed to be an (unsigned) 8bit grayscale image
#define MAX_DEPTHIMAGE_VALUE 255

/**
 * @namespace brics
 */
namespace brics {

/**
 * @class DepthImageToPointCloudTransformation
 * @brief Transforms a depth image into a point cloud
 * @author Sebastian Blumenthal
 * @date Aug 25, 2009
 *
 * TODO: Note that this class needs further implementation regarding intrinsic camera parameters.
 *
 * Results are at the moment <b>just preliminary</b>!
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
	void transformDepthImageToPointCloud(IplImage *depthImage, CartesianPointCloud *pointCloud, double threshold = 0.0);



};

}

#endif /* DEPTHIMAGETOPOINTCLOUDTRANSFORMATION_H_ */

/* EOF */

